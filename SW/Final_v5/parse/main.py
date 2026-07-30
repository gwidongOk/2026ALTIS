import os, threading, queue, serial, serial.tools.list_ports, openpyxl
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from header_parser import parse_header

# ── 글로벌 설정 ──
HERE        = os.path.dirname(os.path.abspath(__file__))
HEADER_PATH = os.path.join(HERE, 'sensor_data.h')
STRUCTS     = parse_header(HEADER_PATH)
EXPECTED_SIZE = {sd.pkt_id: sd.size for sd in STRUCTS.values()}
KNOWN_IDS   = set(EXPECTED_SIZE.keys())
SYNC        = 0xAA
EXCEL_MAX_ROWS = 1_048_576

PHASE_NAMES = {
    0: "PRE_FLIGHT",
    1: "POWERED_FLIGHT",
    2: "COASTING",
    3: "DESCENT",
    4: "LANDED",
}

EVENT_NAMES = {
    1: "LAUNCH",
    2: "BURNOUT",
    3: "APOGEE",
    4: "LANDING",
    5: "NOT_STAGE_CONDITION",
    6: "STAGE2_IGNITION",
    7: "MAIN_DEPLOYMENT",
}


def parse_buffer(buf: bytearray):
    """0xAA 동기화 바이트 기준으로 유효한 패킷만 추출"""
    packets = []
    cursor = 0
    size = len(buf)

    while size - cursor >= 3:
        if buf[cursor] != SYNC:
            cursor += 1
            continue

        pkt_id, pkt_len = buf[cursor + 1], buf[cursor + 2]
        if pkt_id not in KNOWN_IDS or pkt_len != EXPECTED_SIZE.get(pkt_id):
            cursor += 1
            continue

        if size - cursor < pkt_len:
            break

        raw = bytes(buf[cursor:cursor + pkt_len])
        packets.append((pkt_id, raw))
        cursor += pkt_len

    # 한 번만 삭제하여 긴 0xFF 구간이 포함된 32 MB 덤프도 O(n)으로 처리한다.
    if cursor:
        del buf[:cursor]
    return packets


class SerialReader(threading.Thread):
    def __init__(self, port, baud, on_packet, on_raw, on_text):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.on_packet = on_packet
        self.on_raw    = on_raw
        self.on_text   = on_text
        self.running   = False
        self.ser       = None

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode())

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.cancel_read()
            except (AttributeError, OSError):
                pass
            try:
                self.ser.close()
            except OSError:
                pass

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            buf       = bytearray()
            text_line = bytearray()

            while self.running:
                chunk = self.ser.read(4096)
                if not chunk:
                    continue

                buf.extend(chunk)

                # 이진 스트림에서 ASCII 텍스트 라인 병렬 추출
                # (비인쇄 바이트가 오면 현재 라인 리셋 → 바이너리 패킷 내용은 무시됨)
                for b in chunk:
                    if 32 <= b <= 126 or b in (ord('\n'), ord('\r')):
                        text_line.append(b)
                        if b == ord('\n'):
                            line = text_line.decode('ascii', errors='ignore').strip()
                            if line:
                                self.on_text(line)
                            text_line.clear()
                    else:
                        text_line.clear()

                # 이진 패킷 파싱
                for pkt_id, raw in parse_buffer(buf):
                    self.on_raw(raw)
                    sd = STRUCTS.get(pkt_id)
                    if sd:
                        try:
                            self.on_packet(pkt_id, sd.decode(raw))
                        except Exception:
                            pass

        except Exception as e:
            if self.running:
                self.on_text(f"[ERROR] {e}")
        finally:
            self.running = False
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except OSError:
                    pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("2026ALTIS Final_v5 Flight Log Parser")
        self.geometry("1200x720")
        self.packet_queue = queue.Queue()
        self.text_queue   = queue.Queue()
        self.raw_bytes    = bytearray()
        self.all_rows     = []
        self.reader       = None
        self._build_ui()
        self.refresh_ports()
        self._update_ui_loop()

    # ─────────────────────────────────────────────
    # UI 구성
    # ─────────────────────────────────────────────
    def _build_ui(self):
        # ── 상단 연결 바 ──
        frm = ttk.Frame(self, padding=(5, 5, 5, 2))
        frm.pack(fill=tk.X)

        self.cb_port = ttk.Combobox(frm, width=12, state="readonly")
        self.cb_port.pack(side=tk.LEFT, padx=2)

        self.cb_baud = ttk.Combobox(frm, width=8)
        self.cb_baud['values'] = ("115200", "921600")
        self.cb_baud.current(1)
        self.cb_baud.pack(side=tk.LEFT, padx=2)

        ttk.Button(frm, text="Refresh",    command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.btn_conn = ttk.Button(frm, text="Connect", command=self.toggle_connect)
        self.btn_conn.pack(side=tk.LEFT, padx=4)

        self.lbl_cnt = ttk.Label(frm, text="Packets: 0")
        self.lbl_cnt.pack(side=tk.RIGHT, padx=10)

        # ── Final_v5 명령 바 ──
        frm_cmd = ttk.Frame(self, padding=(5, 2, 5, 5))
        frm_cmd.pack(fill=tk.X)

        ttk.Button(frm_cmd, text="ERASE",     command=self.confirm_erase).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="CALIBRATE", command=lambda: self.send_cmd("CALIBRATE")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="DISPLAY",   command=lambda: self.send_cmd("DISPLAY")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="READY",     command=lambda: self.send_cmd("READY")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="STOP",      command=lambda: self.send_cmd("STOP")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="REBOOT",    command=lambda: self.send_cmd("REBOOT")).pack(side=tk.LEFT, padx=2)

        test_menu_btn = ttk.Menubutton(frm_cmd, text="ACTUATOR TEST")
        test_menu = tk.Menu(test_menu_btn, tearoff=False)
        test_menu.add_command(
            label="TEST SERVO",
            command=lambda: self.confirm_actuator_test("TEST SERVO"),
        )
        test_menu.add_command(
            label="TEST PYRO1",
            command=lambda: self.confirm_actuator_test("TEST PYRO1"),
        )
        test_menu.add_command(
            label="TEST PYRO2",
            command=lambda: self.confirm_actuator_test("TEST PYRO2"),
        )
        test_menu_btn["menu"] = test_menu
        test_menu_btn.pack(side=tk.LEFT, padx=2)

        ttk.Separator(frm_cmd, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(frm_cmd, text="PARSE (Dump)", command=self.send_dump).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="Load BIN",     command=self.load_bin).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="Save BIN",     command=self.save_bin).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm_cmd, text="Save XLSX",    command=self.save_xlsx).pack(side=tk.LEFT, padx=2)

        # ── 메인 영역 ──
        frm_main = ttk.Frame(self)
        frm_main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 왼쪽: 패킷 트리뷰
        frm_tree = ttk.Frame(frm_main)
        frm_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.tree = ttk.Treeview(frm_tree, columns=['type', 't', 'val'], show="headings")
        self.tree.heading('type', text='Type')
        self.tree.heading('t',    text='Time (us)')
        self.tree.heading('val',  text='Data')
        self.tree.column('type', width=60,  anchor=tk.CENTER)
        self.tree.column('t',    width=110, anchor='e')
        self.tree.column('val',  width=420)
        vsb = ttk.Scrollbar(frm_tree, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 오른쪽: 펌웨어 응답 로그
        frm_log = ttk.LabelFrame(frm_main, text="Firmware Log", width=290)
        frm_log.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(6, 0))
        frm_log.pack_propagate(False)

        btn_clr = ttk.Button(frm_log, text="Clear", command=self.clear_log)
        btn_clr.pack(anchor=tk.NE, padx=4, pady=2)

        self.log_text = scrolledtext.ScrolledText(frm_log, state=tk.DISABLED,
                                                   font=("Courier", 9), wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # ─────────────────────────────────────────────
    # UI 루프 (100ms 주기)
    # ─────────────────────────────────────────────
    def _update_ui_loop(self):
        try:
            for _ in range(min(self.packet_queue.qsize(), 100)):
                pkt_id, decoded = self.packet_queue.get_nowait()
                self._insert_row(pkt_id, decoded)
        except queue.Empty:
            pass

        try:
            for _ in range(min(self.text_queue.qsize(), 30)):
                line = self.text_queue.get_nowait()
                self._append_log(line)
        except queue.Empty:
            pass

        if (
            self.reader
            and not self.reader.running
            and not self.reader.is_alive()
            and self.btn_conn.cget("text") == "Disconnect"
        ):
            self.btn_conn.config(text="Connect")

        self.after(100, self._update_ui_loop)

    def _insert_row(self, pkt_id, decoded, force_display=False):
        self.all_rows.append((pkt_id, decoded))
        # 센서 데이터는 10개마다 1개만 표시하되, 비행 이벤트는 모두 표시한다.
        if force_display or pkt_id == 6 or len(self.all_rows) % 10 == 0:
            sd = STRUCTS.get(pkt_id)
            type_str    = sd.name if sd else "UNK"
            if pkt_id == 6:
                phase = int(decoded.get("phase", -1))
                event_id = int(decoded.get("event_id", -1))
                display_val = (
                    f"event:{EVENT_NAMES.get(event_id, 'UNKNOWN')}({event_id}), "
                    f"phase:{PHASE_NAMES.get(phase, 'UNKNOWN')}({phase})"
                )
            else:
                display_val = ", ".join(
                    f"{k}:{v}" for k, v in list(decoded.items())[:4]
                )
            self.tree.insert("", tk.END, values=(type_str, decoded.get('t', '0'), display_val))
            if len(self.all_rows) % 200 == 0:
                self.tree.yview_moveto(1.0)
        self.lbl_cnt.config(text=f"Packets: {len(self.all_rows)}")

    def _append_log(self, line: str):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + '\n')
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ─────────────────────────────────────────────
    # 연결 / 명령 전송
    # ─────────────────────────────────────────────
    def toggle_connect(self):
        if self.reader and (self.reader.running or self.reader.is_alive()):
            self.reader.stop()
            self.btn_conn.config(text="Connect")
            self._append_log("[Disconnected]")
        else:
            p, b = self.cb_port.get(), self.cb_baud.get()
            if not p:
                messagebox.showwarning("No Port", "포트를 선택하세요.")
                return
            self.reader = SerialReader(
                p, int(b),
                lambda i, d: self.packet_queue.put((i, d)),
                lambda r: self.raw_bytes.extend(r),
                lambda t: self.text_queue.put(t),
            )
            self.reader.start()
            self.btn_conn.config(text="Disconnect")
            self._append_log(f"[Connected] {p} @ {b}")

    def send_cmd(self, cmd: str):
        if self.reader and self.reader.running:
            self.reader.send(cmd)
            self._append_log(f">> {cmd}")
        else:
            messagebox.showwarning("Not Connected", "먼저 포트에 연결하세요.")

    def send_dump(self):
        if not (self.reader and self.reader.running):
            messagebox.showwarning("Not Connected", "먼저 포트에 연결하세요.")
            return
        self._reset_capture()
        self.send_cmd("PARSE")

    def confirm_erase(self):
        if messagebox.askyesno("ERASE 확인", "Flash 전체를 삭제하시겠습니까?\n(복구 불가)"):
            self.send_cmd("ERASE")

    def confirm_actuator_test(self, cmd: str):
        warnings = {
            "TEST SERVO": "서보가 실제로 움직입니다.",
            "TEST PYRO1": "PYRO1 출력이 실제로 작동합니다.\n점화 장치를 분리했는지 확인하세요.",
            "TEST PYRO2": "PYRO2 출력이 실제로 작동합니다.\n점화 장치를 분리했는지 확인하세요.",
        }
        if messagebox.askyesno(
            f"{cmd} 확인",
            f"{warnings[cmd]}\n\n{cmd} 명령을 전송하시겠습니까?",
        ):
            self.send_cmd(cmd)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port['values'] = ports
        if ports:
            self.cb_port.current(0)

    # ─────────────────────────────────────────────
    # 저장
    # ─────────────────────────────────────────────
    def _reset_capture(self):
        self.raw_bytes.clear()
        self.all_rows.clear()
        self.tree.delete(*self.tree.get_children())
        while True:
            try:
                self.packet_queue.get_nowait()
            except queue.Empty:
                break
        self.lbl_cnt.config(text="Packets: 0")

    def load_bin(self):
        path = filedialog.askopenfilename(
            filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if not path:
            return

        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as e:
            messagebox.showerror("Load failed", f"Binary 파일을 열 수 없습니다:\n{e}")
            return

        self._reset_capture()
        self.raw_bytes.extend(data)

        buf = bytearray(data)
        decoded_count = 0
        decode_errors = 0
        for pkt_id, raw in parse_buffer(buf):
            sd = STRUCTS.get(pkt_id)
            if not sd:
                continue
            try:
                decoded = sd.decode(raw)
            except Exception:
                decode_errors += 1
                continue
            decoded_count += 1
            self._insert_row(pkt_id, decoded, force_display=(decoded_count <= 200))

        skipped = len(buf)
        self._append_log(
            f"[Load BIN] {os.path.basename(path)}: "
            f"{decoded_count} packets, {decode_errors} decode errors, {skipped} trailing bytes"
        )
        messagebox.showinfo(
            "Load BIN",
            f"Binary 로드 완료\n"
            f"패킷: {decoded_count}\n"
            f"디코드 오류: {decode_errors}\n"
            f"남은 바이트: {skipped}"
        )

    def save_bin(self):
        if not self.raw_bytes:
            messagebox.showinfo("Empty", "저장할 데이터가 없습니다.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".bin",
            filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if path:
            with open(path, "wb") as f:
                f.write(self.raw_bytes)
            messagebox.showinfo("Success", f"Binary 저장 완료: {path}")

    def save_xlsx(self):
        if not self.all_rows:
            messagebox.showinfo("Empty", "저장할 패킷이 없습니다.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            filetypes=[("Excel", "*.xlsx"), ("All", "*.*")])
        if not path:
            return
        # write-only 모드로 대용량 덤프를 일반 Workbook보다 적은 RAM으로 저장한다.
        wb = openpyxl.Workbook(write_only=True)
        sheets = {}

        def new_sheet(pkt_id, sd, part):
            suffix = "" if part == 1 else f"_{part}"
            ws = wb.create_sheet(title=f"{sd.name}{suffix}"[:31])
            fields = list(sd.fields)
            if pkt_id == 6:
                fields.extend(("phase_name", "event_name"))
            ws.append(fields)
            return {"ws": ws, "part": part, "rows": 1}

        for pkt_id, decoded in self.all_rows:
            sd = STRUCTS.get(pkt_id)
            if not sd:
                continue
            if pkt_id not in sheets:
                sheets[pkt_id] = new_sheet(pkt_id, sd, 1)

            info = sheets[pkt_id]
            if info["rows"] >= EXCEL_MAX_ROWS:
                info = new_sheet(pkt_id, sd, info["part"] + 1)
                sheets[pkt_id] = info

            row = [decoded.get(f, '') for f in sd.fields]
            if pkt_id == 6:
                phase = int(decoded.get("phase", -1))
                event_id = int(decoded.get("event_id", -1))
                row.extend((
                    PHASE_NAMES.get(phase, "UNKNOWN"),
                    EVENT_NAMES.get(event_id, "UNKNOWN"),
                ))
            info["ws"].append(row)
            info["rows"] += 1

        wb.save(path)
        messagebox.showinfo("Success", f"Excel 저장 완료: {path}")


if __name__ == "__main__":
    App().mainloop()
