"""
HIL all-in-one: OpenRocket simulation → firmware → verification.

Usage
-----
Default (full pipeline from CSV):
    python hil.py HIL.csv
        → flight.hil, dump.bin, flight_*.csv

Subcommands:
    python hil.py convert HIL.csv -o flight.hil
    python hil.py run     flight.hil -o dump.bin
    python hil.py verify  dump.bin --truth HIL.csv -p flight
    python hil.py all     HIL.csv

Options:
    -p, --port COM7              serial port (auto-detect if omitted)
    -b, --baud 921600
    --prefix flight              output file prefix
    --accel-noise 0.025          1-sigma accel noise (m/s^2)
    --gyro-noise  0.005          1-sigma gyro  noise (rad/s)
    --baro-noise  0.30           1-sigma baro  noise (m)
    --seed 42
    --no-erase                   skip flash erase before flight
    --manual-stop                send STOP after fixed delay (no LAND wait)
    --wait-after-start 90        seconds before manual STOP
    --flight-timeout 120         max seconds to wait for STOPPED.
    --summary                    print flight summary after convert
"""

import argparse
import csv
import math
import os
import struct
import sys
import time

# ---- third-party (only required when actually used) ----
try:
    import numpy as np
except ImportError:
    np = None

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


# ============================================================
#  Constants — must match firmware
# ============================================================
ACCEL_SCALE = 0.976e-3 * 9.80665        # m/s^2 per LSB (LSM6DSO32 ±32g)
GYRO_SCALE  = 70.0e-3 * math.pi / 180.0 # rad/s per LSB (LSM6DSO32 ±2000dps)
G = 9.80665

IMU_RATE_HZ = 416.0
BMP_RATE_HZ = 25.0

DEFAULT_ACCEL_NOISE = 0.025
DEFAULT_GYRO_NOISE  = 0.005
DEFAULT_BARO_NOISE  = 0.30

DEFAULT_BAUD = 921600
RESPONSE_TIMEOUT = 30.0
# Must match HIL.cpp: chunk + per-chunk ACK flow control.
# Chunk kept ≤ ESP32-S3 USB-CDC RX queue (256B) so a single chunk fits
# without overrun even before firmware drains the queue.
HIL_CHUNK   = 128
HIL_ACK     = b'\x06'  # ASCII ACK — non-printable, won't collide with text log

# Packet IDs / sizes — must match sensor_data.h
SYNC = 0xAA
ID_BARO, ID_IMU, ID_STATE, ID_EVENT = 1, 2, 5, 6
SZ_BARO, SZ_IMU, SZ_STATE, SZ_EVENT = 11, 19, 47, 9
EXPECTED = {ID_BARO: SZ_BARO, ID_IMU: SZ_IMU, ID_STATE: SZ_STATE, ID_EVENT: SZ_EVENT}
FMT_BARO  = '<BBBIf'
FMT_IMU   = '<BBBI hhh hhh'
FMT_STATE = '<BBBI fff fff ffff'
FMT_EVENT = '<BBBIBB'

PHASE_NAMES = {0: 'PRE_FLIGHT', 1: 'POWERED_FLIGHT', 2: 'COASTING', 3: 'DESCENT', 4: 'LANDED'}
EVENT_NAMES = {1: 'LAUNCH', 2: 'BURNOUT', 3: 'APOGEE', 4: 'LAND', 5: 'NSC'}


# ============================================================
#  Section 1 — CSV → binary trace (convert)
# ============================================================
def _find_col(header, *candidates):
    for cand in candidates:
        for i, h in enumerate(header):
            if cand.lower() in h.lower():
                return i
    return None


def load_openrocket_csv(path):
    if np is None:
        raise SystemExit("numpy required for convert (pip install numpy)")
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    data = np.array([[float(c) if c.strip() else 0.0 for c in r] for r in rows[1:]],
                    dtype=np.float64)

    cols = {
        't':       _find_col(hdr, 'Time'),
        'alt':     _find_col(hdr, 'Altitude (m)'),
        'vvel':    _find_col(hdr, 'Vertical velocity'),
        'vacc':    _find_col(hdr, 'Vertical acceleration'),
        'lacc':    _find_col(hdr, 'Lateral acceleration'),
        'roll':    _find_col(hdr, 'Roll rate'),
        'pitch':   _find_col(hdr, 'Pitch rate'),
        'yaw':     _find_col(hdr, 'Yaw rate'),
        'zenith':  _find_col(hdr, 'Vertical orientation', 'zenith'),
        'azimuth': _find_col(hdr, 'Lateral orientation', 'azimuth'),
    }
    for k in ('t', 'vvel', 'vacc', 'roll', 'pitch', 'yaw', 'zenith'):
        if cols[k] is None:
            raise SystemExit(f"Required column missing: {k}")

    out = {
        't':      data[:, cols['t']],
        'vvel':   data[:, cols['vvel']],
        'vacc':   data[:, cols['vacc']],
        'lacc':   data[:, cols['lacc']]  if cols['lacc'] is not None else np.zeros(len(data)),
        'roll':   data[:, cols['roll']],
        'pitch':  data[:, cols['pitch']],
        'yaw':    data[:, cols['yaw']],
        'zenith': data[:, cols['zenith']],
        'azimuth':data[:, cols['azimuth']] if cols['azimuth'] is not None else np.full(len(data), 90.0),
    }
    if cols['alt'] is not None:
        out['alt'] = data[:, cols['alt']]
    else:
        dt = np.diff(out['t'], prepend=out['t'][0])
        out['alt'] = np.cumsum(out['vvel'] * dt)

    # OpenRocket emits NaN for unresolved fields (e.g. angular rates after decel)
    nan_total = 0
    for k, v in out.items():
        n = int(np.sum(~np.isfinite(v)))
        if n:
            nan_total += n
            out[k] = np.nan_to_num(v, nan=0.0, posinf=0.0, neginf=0.0)
    if nan_total:
        print(f"  Replaced {nan_total} NaN/inf values with 0")
    return out


def world_to_body_accel(vacc, lacc, zenith_deg):
    """
    World (vert, lat) -> body (ax, ay, az) using nose-pitch-plane model.
    Assumes lateral acceleration lies in body XY plane (no side-slip).

    theta = pi/2 - zenith  (tilt from vertical, 0=upright)
    Body X = nose = world (sin theta, 0, cos theta)
    Body Z = down = world (cos theta, 0, -sin theta)
    f_world = (lat, 0, vert + g)   (specific force, up positive)

    Static check (theta=0, vert=lat=0): ax = g, ay = 0, az = 0
    Free fall (vert=-g): ax = 0
    """
    theta = np.pi / 2.0 - np.deg2rad(zenith_deg)
    f_vert = vacc + G
    ax = np.cos(theta) * f_vert + np.sin(theta) * lacc
    ay = np.zeros_like(ax)
    az = np.cos(theta) * lacc - np.sin(theta) * f_vert
    return ax, ay, az


def _resample(t_src, y_src, t_dst):
    return np.interp(t_dst, t_src, y_src)


def _clip_i16(x):
    return np.clip(np.round(x), -32768, 32767).astype(np.int16)


def build_trace(d, accel_n, gyro_n, baro_n, seed):
    t = d['t']
    t_end = t[-1]
    ax, ay, az = world_to_body_accel(d['vacc'], d['lacc'], d['zenith'])
    gx = np.deg2rad(d['roll'])
    gy = np.deg2rad(d['pitch'])
    gz = np.deg2rad(d['yaw'])

    rng = np.random.default_rng(seed)
    n_imu = int(t_end * IMU_RATE_HZ)
    t_imu = np.arange(n_imu) / IMU_RATE_HZ
    n_bmp = int(t_end * BMP_RATE_HZ)
    t_bmp = np.arange(n_bmp) / BMP_RATE_HZ

    def rs_noise(y, sigma, t_dst, n):
        return _resample(t, y, t_dst) + rng.normal(0, sigma, n)

    return {
        't_imu': t_imu,
        'gx': _clip_i16(rs_noise(gx, gyro_n,  t_imu, n_imu) / GYRO_SCALE),
        'gy': _clip_i16(rs_noise(gy, gyro_n,  t_imu, n_imu) / GYRO_SCALE),
        'gz': _clip_i16(rs_noise(gz, gyro_n,  t_imu, n_imu) / GYRO_SCALE),
        'ax': _clip_i16(rs_noise(ax, accel_n, t_imu, n_imu) / ACCEL_SCALE),
        'ay': _clip_i16(rs_noise(ay, accel_n, t_imu, n_imu) / ACCEL_SCALE),
        'az': _clip_i16(rs_noise(az, accel_n, t_imu, n_imu) / ACCEL_SCALE),
        't_bmp': t_bmp,
        'alt': rs_noise(d['alt'], baro_n, t_bmp, n_bmp).astype(np.float32),
    }


def write_trace(trace, out_path):
    """Binary layout: "HILT"(4) + imu_count(u32) + bmp_count(u32) + reserved(u32)
       + imu_count × (dt_us(u32) + 6×i16) + bmp_count × (dt_us(u32) + f32)
    """
    n_imu = len(trace['t_imu'])
    n_bmp = len(trace['t_bmp'])
    dt_imu = np.diff(trace['t_imu'], prepend=0.0) * 1e6
    dt_bmp = np.diff(trace['t_bmp'], prepend=0.0) * 1e6
    dt_imu[0] = 2400
    dt_bmp[0] = 40000

    with open(out_path, 'wb') as f:
        f.write(b'HILT')
        f.write(struct.pack('<III', n_imu, n_bmp, 0))
        imu_rec = struct.Struct('<Ihhhhhh')
        for i in range(n_imu):
            f.write(imu_rec.pack(int(dt_imu[i]),
                                 int(trace['gx'][i]), int(trace['gy'][i]), int(trace['gz'][i]),
                                 int(trace['ax'][i]), int(trace['ay'][i]), int(trace['az'][i])))
        bmp_rec = struct.Struct('<If')
        for i in range(n_bmp):
            f.write(bmp_rec.pack(int(dt_bmp[i]), float(trace['alt'][i])))
    return n_imu, n_bmp


def print_summary(d):
    t, v, a, z, vacc = d['t'], d['vvel'], d['alt'], d['zenith'], d['vacc']
    i_max_alt = int(np.argmax(a))
    i_max_v   = int(np.argmax(v))
    i_launch  = int(np.argmax(np.abs(vacc) > 1))
    print(f"\nFlight summary (ground truth):")
    print(f"  Launch        : t~{t[i_launch]:.3f}s")
    print(f"  Peak velocity : t={t[i_max_v]:.3f}s  v={v[i_max_v]:.1f} m/s  alt={a[i_max_v]:.1f} m")
    print(f"  Apogee        : t={t[i_max_alt]:.3f}s  alt={a[i_max_alt]:.1f} m  zenith={z[i_max_alt]:.1f} deg")
    print(f"  Touchdown     : t={t[-1]:.3f}s  alt={a[-1]:.1f} m  vvel={v[-1]:.2f} m/s")


def do_convert(csv_path, out_path, accel_n, gyro_n, baro_n, seed, summary=False):
    print(f"[convert] {csv_path} -> {out_path}")
    d = load_openrocket_csv(csv_path)
    print(f"  rows={len(d['t'])}, t_end={d['t'][-1]:.2f}s")
    print(f"  noise: accel={accel_n} m/s^2, gyro={gyro_n} rad/s, baro={baro_n} m")
    trace = build_trace(d, accel_n, gyro_n, baro_n, seed)
    n_imu, n_bmp = write_trace(trace, out_path)
    size = os.path.getsize(out_path)
    print(f"  IMU={n_imu} ({n_imu*16}B)  BMP={n_bmp} ({n_bmp*8}B)  total={size/1024:.1f} KB")
    if summary:
        print_summary(d)


# ============================================================
#  Section 2 — Serial runner (run)
# ============================================================
def _autodetect_port():
    if serial is None:
        return None
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        desc = (p.description or '').lower()
        if 'usb' in desc or 'cdc' in desc or 'serial' in desc:
            return p.device
    return ports[0].device


def _wait_for(ser, needle, timeout=RESPONSE_TIMEOUT, echo=True):
    buf = b''
    deadline = time.time() + timeout
    lines = []
    while time.time() < deadline:
        chunk = ser.read(1024)
        if chunk:
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                s = line.decode('ascii', errors='replace').strip()
                if s:
                    lines.append(s)
                    if echo:
                        print(f"  < {s}")
                    if needle in s:
                        return lines
        else:
            time.sleep(0.01)
    raise TimeoutError(f"Did not see '{needle}' within {timeout}s")


def _send_cmd(ser, cmd, echo=True):
    if echo:
        print(f"  > {cmd}")
    ser.write((cmd + '\n').encode())
    ser.flush()


def _wait_ack(ser, timeout=5.0):
    """Read bytes until ACK byte arrives. Other bytes (text log) are discarded."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == HIL_ACK:
            return True
        # Otherwise ignore (e.g. firmware text log byte) and keep waiting
    return False


def _upload_trace(ser, trace_path):
    with open(trace_path, 'rb') as f:
        data = f.read()
    size = len(data)
    print(f"  Uploading {trace_path} ({size/1024:.1f} KB, chunk={HIL_CHUNK}) ...")
    t0 = time.time()

    # 1. Send 16-byte header
    HDR = 16
    ser.write(data[:HDR]); ser.flush()

    # 2. Wait for header ACK (also signals firmware allocated PSRAM)
    if not _wait_ack(ser, timeout=10):
        raise IOError("HIL header ACK not received (alloc failed?)")

    # 3. Stream rest in chunks, await ACK after each
    pos = HDR
    while pos < size:
        n = min(HIL_CHUNK, size - pos)
        ser.write(data[pos:pos+n]); ser.flush()
        if not _wait_ack(ser, timeout=5):
            raise IOError(f"HIL chunk ACK timeout at offset {pos}")
        pos += n
        if pos % (32 * 1024) == 0 or pos == size:
            rate = (pos - HDR) / (time.time() - t0) / 1024
            print(f"    {pos}/{size} B  ({rate:.1f} KB/s)")
    print(f"  Upload done in {time.time()-t0:.2f}s")


def _collect_dump(ser, out_path, idle_timeout=2.0):
    print(f"  Capturing dump to {out_path} ...")
    buf = bytearray()
    state = 'wait_start'
    deadline = time.time() + 60.0
    last_data = time.time()
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
            last_data = time.time()
            deadline = time.time() + 60.0
        else:
            if state == 'wait_start' and time.time() > deadline:
                raise TimeoutError("DUMP START not received")
            if state == 'capturing' and (time.time() - last_data) > idle_timeout:
                break
            time.sleep(0.01)
            continue

        if state == 'wait_start':
            idx = buf.find(b'DUMP START')
            if idx >= 0:
                nl = buf.find(b'\n', idx)
                if nl >= 0:
                    del buf[:nl+1]
                    state = 'capturing'
                    print("    DUMP START received, capturing...")

        if state == 'capturing':
            idx = buf.find(b'DUMP DONE')
            if idx >= 0:
                payload = bytes(buf[:idx]).rstrip(b'\r\n')
                with open(out_path, 'wb') as f:
                    f.write(payload)
                print(f"  DUMP DONE: {len(payload)} bytes")
                return out_path

    if state == 'capturing':
        with open(out_path, 'wb') as f:
            f.write(bytes(buf))
        print(f"  (idle timeout) saved {len(buf)} bytes")
        return out_path
    raise RuntimeError("Dump capture failed")


def do_run(trace_path, dump_path, port, baud, no_erase, manual_stop,
           wait_after_start, flight_timeout):
    if serial is None:
        raise SystemExit("pyserial required for run (pip install pyserial)")
    port = port or _autodetect_port()
    if not port:
        raise SystemExit("No serial port found")
    print(f"[run] port={port} @ {baud}")
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()
    try:
        if not no_erase:
            _send_cmd(ser, "ERASE")
            print("  (chip erase 1-3 min for 32MB)")
            _wait_for(ser, "DONE.", timeout=300)

        _send_cmd(ser, "HIL")
        _wait_for(ser, "WAITING FOR TRACE")
        _upload_trace(ser, trace_path)
        _wait_for(ser, "HIL READY", timeout=60)

        _send_cmd(ser, "CALIBRATE")
        _wait_for(ser, "CALIBRATION SKIPPED", timeout=5)

        _send_cmd(ser, "START")
        _wait_for(ser, "FLIGHT ACTIVE")

        if manual_stop:
            print(f"  Waiting {wait_after_start}s before STOP ...")
            time.sleep(wait_after_start)
            _send_cmd(ser, "STOP")
            _wait_for(ser, "STOPPED.", timeout=10)
        else:
            try:
                _wait_for(ser, "STOPPED.", timeout=flight_timeout)
            except TimeoutError:
                print("  TIMEOUT — sending STOP")
                _send_cmd(ser, "STOP")
                _wait_for(ser, "STOPPED.", timeout=10)

        _send_cmd(ser, "PARSE")
        _collect_dump(ser, dump_path)
        print(f"[run] OK -> {dump_path}")
    finally:
        ser.close()


# ============================================================
#  Section 3 — Dump verifier (verify)
# ============================================================
def parse_dump(raw):
    out = {ID_BARO: [], ID_IMU: [], ID_STATE: [], ID_EVENT: []}
    buf = memoryview(raw)
    i = 0
    skipped = 0
    while i + 3 <= len(buf):
        if buf[i] != SYNC:
            i += 1; skipped += 1; continue
        pid, plen = buf[i+1], buf[i+2]
        if pid not in EXPECTED or plen != EXPECTED[pid]:
            i += 1; skipped += 1; continue
        if i + plen > len(buf):
            break
        pkt = bytes(buf[i:i+plen])
        if pid == ID_BARO:
            _,_,_,t,alt = struct.unpack(FMT_BARO, pkt)
            out[pid].append((t, alt))
        elif pid == ID_IMU:
            _,_,_,t,gx,gy,gz,ax,ay,az = struct.unpack(FMT_IMU, pkt)
            out[pid].append((t, gx, gy, gz, ax, ay, az))
        elif pid == ID_STATE:
            vals = struct.unpack(FMT_STATE, pkt)
            out[pid].append(vals[3:])  # drop header(3)
        elif pid == ID_EVENT:
            _,_,_,t,phase,eid = struct.unpack(FMT_EVENT, pkt)
            out[pid].append((t, phase, eid))
        i += plen
    print(f"  BARO={len(out[ID_BARO])} IMU={len(out[ID_IMU])} STATE={len(out[ID_STATE])} EVENT={len(out[ID_EVENT])} (skipped {skipped} B)")
    return out


def _write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"  wrote {path} ({len(rows)} rows)")


def _normalize_time(rows):
    if not rows:
        return rows
    t0 = rows[0][0]
    return [(((r[0]-t0) & 0xFFFFFFFF) * 1e-6,) + r[1:] for r in rows]


def _load_truth(csv_path):
    if np is None:
        raise SystemExit("numpy required for verify")
    with open(csv_path, 'r', encoding='utf-8', errors='replace') as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    def col(*names):
        for n in names:
            for i, h in enumerate(hdr):
                if n.lower() in h.lower():
                    return i
        return None
    idx_t  = col('Time')
    idx_a  = col('Altitude (m)')
    idx_v  = col('Vertical velocity')
    idx_z  = col('Vertical orientation', 'zenith')
    data = np.array([[float(c) if c.strip() else 0.0 for c in r] for r in rows[1:]],
                    dtype=np.float64)
    data = np.nan_to_num(data)
    return {
        't':      data[:, idx_t],
        'alt':    data[:, idx_a] if idx_a is not None else np.zeros(len(data)),
        'vvel':   data[:, idx_v] if idx_v is not None else np.zeros(len(data)),
        'zenith': data[:, idx_z] if idx_z is not None else np.full(len(data), 90.0),
    }


def _compare(nav_rows, truth, out_path):
    if not nav_rows:
        print("  (no STATE packets to compare)")
        return None
    arr = np.array(nav_rows)
    t = arr[:, 0]
    alt_kf  = -arr[:, 3]
    vvel_kf = -arr[:, 6]
    qw, qx, qy, qz = arr[:, 7], arr[:, 8], arr[:, 9], arr[:, 10]
    cos_tilt_kf = 2.0 * (qw*qy - qx*qz)

    alt_t  = np.interp(t, truth['t'], truth['alt'])
    vvel_t = np.interp(t, truth['t'], truth['vvel'])
    cos_t  = np.sin(np.deg2rad(np.interp(t, truth['t'], truth['zenith'])))

    err_a = alt_kf - alt_t
    err_v = vvel_kf - vvel_t
    err_c = cos_tilt_kf - cos_t

    rmse_a = float(np.sqrt(np.mean(err_a**2)))
    rmse_v = float(np.sqrt(np.mean(err_v**2)))
    rmse_c = float(np.sqrt(np.mean(err_c**2)))
    print("\n=== Algorithm accuracy vs OpenRocket ===")
    print(f"  Altitude  RMSE: {rmse_a:>7.2f} m   (max {float(np.max(np.abs(err_a))):.2f} m)")
    print(f"  Vert vel  RMSE: {rmse_v:>7.2f} m/s (max {float(np.max(np.abs(err_v))):.2f} m/s)")
    print(f"  cos(tilt) RMSE: {rmse_c:>7.3f}    (~{math.degrees(math.asin(min(1, rmse_c))):.1f}° angular)")

    rows = []
    for i in range(len(t)):
        rows.append([f"{t[i]:.4f}",
                     f"{alt_kf[i]:.3f}",  f"{alt_t[i]:.3f}",  f"{err_a[i]:.3f}",
                     f"{vvel_kf[i]:.3f}", f"{vvel_t[i]:.3f}", f"{err_v[i]:.3f}",
                     f"{cos_tilt_kf[i]:.4f}", f"{cos_t[i]:.4f}", f"{err_c[i]:.4f}"])
    _write_csv(out_path,
               ['t','alt_kf','alt_truth','err_alt',
                'vvel_kf','vvel_truth','err_vvel',
                'cos_tilt_kf','cos_tilt_truth','err_cos_tilt'],
               rows)


def _print_events(events):
    if not events:
        print("  (no EVENT packets)")
        return
    print("\n=== FSM events (firmware) ===")
    print(f"  {'t [s]':>8}  {'phase':<15} event")
    for t, phase, eid in events:
        print(f"  {t:>8.3f}  {PHASE_NAMES.get(phase,'?'):<15} {EVENT_NAMES.get(eid,'?')} ({eid})")


def do_verify(dump_path, prefix, truth_path=None):
    print(f"[verify] {dump_path} -> {prefix}_*.csv")
    with open(dump_path, 'rb') as f:
        raw = f.read()
    print(f"  Loaded {len(raw)} B")
    parsed = parse_dump(raw)

    state_rows = _normalize_time(parsed[ID_STATE])
    baro_rows  = _normalize_time(parsed[ID_BARO])
    imu_rows   = _normalize_time(parsed[ID_IMU])
    event_rows = _normalize_time(parsed[ID_EVENT])

    _write_csv(f"{prefix}_nav.csv",
               ['t','pN','pE','pD','vN','vE','vD','qw','qx','qy','qz'],
               [[f"{r[0]:.4f}"] + [f"{v:.6f}" for v in r[1:]] for r in state_rows])
    _write_csv(f"{prefix}_baro.csv",
               ['t','alt'],
               [[f"{r[0]:.4f}", f"{r[1]:.3f}"] for r in baro_rows])
    _write_csv(f"{prefix}_imu.csv",
               ['t','gx','gy','gz','ax','ay','az'],
               [[f"{r[0]:.4f}"] + list(r[1:]) for r in imu_rows])
    _write_csv(f"{prefix}_events.csv",
               ['t','phase','event_id','phase_name','event_name'],
               [[f"{r[0]:.4f}", r[1], r[2],
                 PHASE_NAMES.get(r[1],'?'), EVENT_NAMES.get(r[2],'?')] for r in event_rows])

    _print_events(event_rows)

    if truth_path:
        truth = _load_truth(truth_path)
        _compare(state_rows, truth, f"{prefix}_compare.csv")


# ============================================================
#  Main — subcommand routing (+ auto "all" for .csv input)
# ============================================================
def build_parser():
    ap = argparse.ArgumentParser(prog='hil.py',
        description='HIL: OpenRocket -> firmware -> verify (all-in-one)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = ap.add_subparsers(dest='cmd')

    # convert
    p = sub.add_parser('convert', help='OpenRocket CSV -> binary trace')
    p.add_argument('csv')
    p.add_argument('-o', '--out', default='flight.hil')
    p.add_argument('--accel-noise', type=float, default=DEFAULT_ACCEL_NOISE)
    p.add_argument('--gyro-noise',  type=float, default=DEFAULT_GYRO_NOISE)
    p.add_argument('--baro-noise',  type=float, default=DEFAULT_BARO_NOISE)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--summary', action='store_true')

    # run
    p = sub.add_parser('run', help='upload trace, drive flight, dump flash')
    p.add_argument('trace')
    p.add_argument('-o', '--dump', default='dump.bin')
    p.add_argument('-p', '--port')
    p.add_argument('-b', '--baud', type=int, default=DEFAULT_BAUD)
    p.add_argument('--no-erase', action='store_true')
    p.add_argument('--manual-stop', action='store_true')
    p.add_argument('--wait-after-start', type=float, default=90.0)
    p.add_argument('--flight-timeout', type=float, default=120.0)

    # verify
    p = sub.add_parser('verify', help='parse dump -> CSVs + compare with truth')
    p.add_argument('dump')
    p.add_argument('--truth')
    p.add_argument('-p', '--prefix', default='flight')

    # all
    p = sub.add_parser('all', help='convert + run + verify (full pipeline)')
    p.add_argument('csv')
    p.add_argument('--prefix', default='flight')
    p.add_argument('-p', '--port')
    p.add_argument('-b', '--baud', type=int, default=DEFAULT_BAUD)
    p.add_argument('--accel-noise', type=float, default=DEFAULT_ACCEL_NOISE)
    p.add_argument('--gyro-noise',  type=float, default=DEFAULT_GYRO_NOISE)
    p.add_argument('--baro-noise',  type=float, default=DEFAULT_BARO_NOISE)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--no-erase', action='store_true')
    p.add_argument('--manual-stop', action='store_true')
    p.add_argument('--wait-after-start', type=float, default=90.0)
    p.add_argument('--flight-timeout', type=float, default=120.0)
    p.add_argument('--summary', action='store_true')

    return ap


def main():
    # Shortcut: `python hil.py HIL.csv ...` -> implicit "all"
    if len(sys.argv) >= 2 and sys.argv[1].lower().endswith('.csv'):
        sys.argv.insert(1, 'all')

    ap = build_parser()
    args = ap.parse_args()

    if args.cmd == 'convert':
        do_convert(args.csv, args.out, args.accel_noise, args.gyro_noise,
                   args.baro_noise, args.seed, args.summary)
    elif args.cmd == 'run':
        do_run(args.trace, args.dump, args.port, args.baud, args.no_erase,
               args.manual_stop, args.wait_after_start, args.flight_timeout)
    elif args.cmd == 'verify':
        do_verify(args.dump, args.prefix, args.truth)
    elif args.cmd == 'all':
        trace = f"{args.prefix}.hil"
        dump  = f"{args.prefix}.bin"
        do_convert(args.csv, trace, args.accel_noise, args.gyro_noise,
                   args.baro_noise, args.seed, args.summary)
        do_run(trace, dump, args.port, args.baud, args.no_erase,
               args.manual_stop, args.wait_after_start, args.flight_timeout)
        do_verify(dump, args.prefix, args.csv)
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
