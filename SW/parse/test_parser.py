"""
test_parser.py -- Parser unit test (no board required)

1. Build synthetic binary packets for all 4 types (BARO/IMU/STATE/EVENT)
2. Run through parse_buffer
3. Compare against expected values
4. All PASS = parser is working correctly

Usage:
    python test_parser.py
"""
import struct, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from header_parser import parse_header

# -- Init parser --
STRUCTS     = parse_header(os.path.join(HERE, 'sensor_data.h'))
EXPECTED_SZ = {sd.pkt_id: sd.size for sd in STRUCTS.values()}
KNOWN_IDS   = set(EXPECTED_SZ)
SYNC        = 0xAA

def parse_buffer(buf: bytearray):
    packets = []
    while len(buf) >= 3:
        if buf[0] != SYNC:
            buf.pop(0); continue
        pid  = buf[1]
        plen = buf[2]
        if pid not in KNOWN_IDS or plen != EXPECTED_SZ.get(pid):
            buf.pop(0); continue
        if len(buf) < plen:
            break
        raw = bytes(buf[:plen])
        del buf[:plen]
        packets.append((pid, raw))
    return packets

# -- Packet builders --
def make_baro(t, alt):
    return struct.pack('<BBBIf', SYNC, 1, 11, t, alt)

def make_imu(t, gx, gy, gz, ax, ay, az):
    return struct.pack('<BBBIhhhhhh', SYNC, 2, 19, t, gx, gy, gz, ax, ay, az)

def make_state(t, pN, pE, pD, vN, vE, vD, qw, qx, qy, qz):
    return struct.pack('<BBBI' + 'f'*10,
                      SYNC, 5, 47, t,
                      pN, pE, pD, vN, vE, vD, qw, qx, qy, qz)

def make_event(t, phase, event_id):
    return struct.pack('<BBBIBB', SYNC, 6, 9, t, phase, event_id)

# -- Test cases --
tests = [
    {
        'name'  : 'BARO  (ID=1)',
        'raw'   : make_baro(500000, 97.5),
        'pid'   : 1,
        'expect': {'t': 500000, 'alt': 97.5},
    },
    {
        'name'  : 'IMU   (ID=2)',
        'raw'   : make_imu(1000000, 10, -5, 3, 1024, 0, -2),
        'pid'   : 2,
        'expect': {'t': 1000000,
                   'gx': 10, 'gy': -5, 'gz': 3,
                   'ax': 1024, 'ay': 0, 'az': -2},
    },
    {
        'name'  : 'STATE (ID=5)',
        'raw'   : make_state(2000000,
                             0.0, 0.0, -100.0,
                             0.0, 0.0, -4.0,
                             0.707, 0.0, 0.707, 0.0),
        'pid'   : 5,
        'expect': {'t': 2000000,
                   'pN': 0.0, 'pE': 0.0, 'pD': -100.0,
                   'vN': 0.0, 'vE': 0.0, 'vD': -4.0},
    },
    {
        'name'  : 'EVENT (ID=6) LAUNCH',
        'raw'   : make_event(3000000, 1, 1),
        'pid'   : 6,
        'expect': {'t': 3000000, 'phase': 1, 'event_id': 1},
    },
    {
        'name'  : 'EVENT (ID=6) APOGEE',
        'raw'   : make_event(4000000, 2, 3),
        'pid'   : 6,
        'expect': {'t': 4000000, 'phase': 2, 'event_id': 3},
    },
]

# -- Full dump with noise --
def make_full_dump():
    dump = bytearray()
    dump += b'\x00\x00\x00'
    dump += make_baro(100000, 0.5)
    dump += make_imu(100000, 0, 0, 0, 1024, 0, 0)
    dump += b'\xAA\xFF'                         # false sync -- must be skipped
    dump += make_state(200000, 0, 0, -50, 0, 0, -10, 1, 0, 0, 0)
    dump += make_event(300000, 1, 1)             # LAUNCH
    dump += make_event(310000, 2, 2)             # BO
    dump += make_event(320000, 2, 3)             # APOGEE
    dump += make_event(330000, 3, 4)             # LAND
    dump += b'\xFF\xFF\xFF\xFF'
    return dump

# -- Run tests --
print('=' * 56)
print(' 2026ALTIS parser unit test')
print('=' * 56)

all_pass = True

print('\n[1] Individual packet parsing')
for tc in tests:
    buf     = bytearray(tc['raw'])
    packets = parse_buffer(buf)

    if not packets:
        print(f"  FAIL  {tc['name']} -- no packet parsed")
        all_pass = False
        continue

    pid, raw = packets[0]
    if pid != tc['pid']:
        print(f"  FAIL  {tc['name']} -- pid mismatch {pid}")
        all_pass = False
        continue

    sd      = STRUCTS[pid]
    decoded = sd.decode(raw)
    fail    = False
    for k, exp in tc['expect'].items():
        got = decoded.get(k)
        if got is None or abs(float(got) - float(exp)) > 0.01:
            print(f"  FAIL  {tc['name']} -- {k}: got={got}, expected={exp}")
            fail = True
    if not fail:
        print(f"  PASS  {tc['name']}")
    else:
        all_pass = False

print('\n[2] Full dump parsing (with garbage bytes)')
dump    = make_full_dump()
buf     = bytearray(dump)
packets = parse_buffer(buf)

expected_pids = [1, 2, 5, 6, 6, 6, 6]
parsed_pids   = [pid for pid, _ in packets]

if parsed_pids == expected_pids:
    print(f'  PASS  {len(packets)} packets parsed, garbage ignored')
    print(f'        order: {parsed_pids}')
else:
    print(f'  FAIL  expected={expected_pids}, got={parsed_pids}')
    all_pass = False

print('\n[3] Packet size table')
firmware_sizes = {1: 11, 2: 19, 5: 47, 6: 9}
names = {1: 'baro_pkt', 2: 'imu_pkt', 5: 'state_pkt', 6: 'event_pkt'}
for pid, exp in firmware_sizes.items():
    if pid in STRUCTS:
        actual = STRUCTS[pid].size
        ok     = actual == exp
        mark   = 'PASS' if ok else 'FAIL'
        if not ok: all_pass = False
        print(f'  {mark}  ID={pid} {names[pid]:12s}  {actual:3d}B  (firmware expects {exp}B)')
        print(f'        fields: {STRUCTS[pid].fields}')
    else:
        print(f'  FAIL  ID={pid} -- not found in header')
        all_pass = False

# Save test dump file
dump_path = os.path.join(HERE, 'test_dump.bin')
with open(dump_path, 'wb') as f:
    f.write(make_full_dump())
print(f'\n[4] Test dump saved: {dump_path}')
print( '     Load in main.py: connect serial -> receive dump -> Save BIN -> verify')

print()
print('=' * 56)
if all_pass:
    print(' ALL PASS -- parser is working correctly')
else:
    print(' SOME FAILED -- check errors above')
print('=' * 56)
