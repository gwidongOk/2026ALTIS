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

def parse_buffer(buf: bytearray):
    """0xAA 동기화 바이트 기준으로 유효한 패킷만 추출"""
    packets = []
    while len(buf) >= 3:
        if buf[0] != SYNC:
            buf.pop(0)
            continue
        pkt_id, pkt_len = buf[1], buf[2]
        if pkt_id not in KNOWN_IDS or pkt_len != EXPECTED_SIZE.get(pkt_id):
            buf.pop(0)
            continue
        if len(buf) < pkt_len:
            break
        raw = bytes(buf[:pkt_len])
        del buf[:pkt_len]
        packets.append((pkt_id, raw))
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
            self.on_text(f"[ERROR] {e}")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Avionics Noise Analyzer")
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
        # ── 상단 바 ──
        frm = ttk.Frame(self, padding=5)
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

        ttk.Separator(frm, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(frm, text="CALIBRATE", command=lambda: self.send_cmd("CALIBRATE")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm, text="START",     command=lambda: self.send_cmd("START")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm, text="STOP",      command=lambda: self.send_cmd("STOP")).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm, text="ERASE",     command=self.confirm_erase).pack(side=tk.LEFT, padx=2)

        ttk.Separator(frm, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(frm, text="PARSE (Dump)", command=self.send_dump).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm, text="Save BIN",     command=self.save_bin).pack(side=tk.LEFT, padx=2)
        ttk.Button(frm, text="Save XLSX",    command=self.save_xlsx).pack(side=tk.LEFT, padx=2)

        self.lbl_cnt = ttk.Label(frm, text="Packets: 0")
        self.lbl_cnt.pack(side=tk.RIGHT, padx=10)

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
        self.tree.column('t',    width=110, anchor=tk.RIGHT)
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

        self.after(100, self._update_ui_loop)

    def _insert_row(self, pkt_id, decoded):
        self.all_rows.append((pkt_id, decoded))
        # 10개마다 1개씩 표시 (대량 데이터 성능 보호)
        if len(self.all_rows) % 10 == 0:
            sd = STRUCTS.get(pkt_id)
            type_str    = sd.name if sd else "UNK"
            display_val = ", ".join(f"{k}:{v}" for k, v in list(decoded.items())[:4])
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
        if self.reader and self.reader.running:
            self.reader.running = False
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
        self.send_cmd("PARSE")

    def confirm_erase(self):
        if messagebox.askyesno("ERASE 확인", "Flash 전체를 삭제하시겠습니까?\n(복구 불가)"):
            self.send_cmd("ERASE")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port['values'] = ports
        if ports:
            self.cb_port.current(0)

    # ─────────────────────────────────────────────
    # 저장
    # ─────────────────────────────────────────────
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
        wb = openpyxl.Workbook()
        sheets = {}
        for pkt_id, decoded in self.all_rows:
            sd = STRUCTS.get(pkt_id)
            if not sd:
                continue
            if pkt_id not in sheets:
                ws = wb.create_sheet(title=sd.name)
                ws.append(sd.fields)
                sheets[pkt_id] = ws
            sheets[pkt_id].append([decoded.get(f, '') for f in sd.fields])
        if 'Sheet' in wb.sheetnames:
            del wb['Sheet']
        wb.save(path)
        messagebox.showinfo("Success", f"Excel 저장 완료: {path}")


if __name__ == "__main__":
    App().mainloop()
