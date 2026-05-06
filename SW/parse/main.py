import os, sys, threading, queue, serial, serial.tools.list_ports, openpyxl
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from header_parser import parse_header

# ── 글로벌 설정 ──
HERE = os.path.dirname(os.path.abspath(__file__))
HEADER_PATH = os.path.join(HERE, 'sensor_data.h')
STRUCTS = parse_header(HEADER_PATH)
EXPECTED_SIZE = {sd.pkt_id: sd.size for sd in STRUCTS.values()}
KNOWN_IDS = set(EXPECTED_SIZE.keys())
SYNC = 0xAA

def parse_buffer(buf: bytearray):
    packets = []
    while len(buf) >= 3:
        if buf[0] != SYNC:
            buf.pop(0)
            continue
        pkt_id, pkt_len = buf[1], buf[2]
        if pkt_id not in KNOWN_IDS or pkt_len != EXPECTED_SIZE.get(pkt_id):
            buf.pop(0)
            continue
        if len(buf) < pkt_len: break
        raw = bytes(buf[:pkt_len])
        del buf[:pkt_len]
        packets.append((pkt_id, raw))
    return packets

class SerialReader(threading.Thread):
    def __init__(self, port, baud, on_packet, on_raw):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.on_packet, self.on_raw = on_packet, on_raw
        self.running = False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            buf = bytearray()
            while self.running:
                chunk = self.ser.read(4096)
                if chunk:
                    buf.extend(chunk)
                    for pkt_id, raw in parse_buffer(buf):
                        self.on_raw(raw)
                        sd = STRUCTS.get(pkt_id)
                        if sd:
                            try:
                                self.on_packet(pkt_id, sd.decode(raw))
                            except: pass
        except Exception as e:
            print(f"Serial Error: {e}")

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Avionics Data Analyzer")
        self.geometry("1100x700")
        self.packet_queue = queue.Queue()
        self.raw_bytes = bytearray()
        self.all_rows = []
        self._build_ui()
        self.refresh_ports()
        self._update_ui_loop()

    def _build_ui(self):
        frm_top = ttk.Frame(self, padding=5)
        frm_top.pack(fill=tk.X)

        self.cb_port = ttk.Combobox(frm_top, width=15, state="readonly")
        self.cb_port.pack(side=tk.LEFT, padx=5)
        
        self.cb_baud = ttk.Combobox(frm_top, width=10)
        self.cb_baud['values'] = ("115200", "921600")
        self.cb_baud.current(1)
        self.cb_baud.pack(side=tk.LEFT, padx=5)

        ttk.Button(frm_top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        self.btn_conn = ttk.Button(frm_top, text="Connect", command=self.toggle_connect)
        self.btn_conn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(frm_top, text="Dump (P)", command=self.send_dump).pack(side=tk.LEFT)
        ttk.Button(frm_top, text="Save BIN", command=self.save_bin).pack(side=tk.RIGHT, padx=5)
        ttk.Button(frm_top, text="Save XLSX", command=self.save_xlsx).pack(side=tk.RIGHT, padx=5)
        
        self.lbl_cnt = ttk.Label(frm_top, text="Packets: 0")
        self.lbl_cnt.pack(side=tk.RIGHT, padx=10)

        # Treeview 설정 (ID별로 필드가 다르므로 공통 필드 위주 구성)
        self.tree = ttk.Treeview(self, columns=['type', 't', 'val'], show="headings")
        self.tree.heading('type', text='Type'); self.tree.heading('t', text='Time'); self.tree.heading('val', text='Data')
        self.tree.pack(fill=tk.BOTH, expand=True)

    def _update_ui_loop(self):
        """큐를 비우며 UI 업데이트 (초당 10번 실행)"""
        try:
            for _ in range(min(self.packet_queue.qsize(), 100)):
                pkt_id, decoded = self.packet_queue.get_nowait()
                self._insert_row(pkt_id, decoded)
        except queue.Empty: pass
        self.after(100, self._update_ui_loop)

    def _insert_row(self, pkt_id, decoded):
        self.all_rows.append((pkt_id, decoded))
        if len(self.all_rows) % 10 == 0: # 10개마다 하나씩만 화면에 표시 (성능 최적화)
            sd = STRUCTS.get(pkt_id)
            type_str = sd.name if sd else "UNK"
            # 가독성을 위해 첫 3개 필드만 표시
            display_val = ", ".join([f"{k}:{v}" for k, v in list(decoded.items())[:3]])
            self.tree.insert("", tk.END, values=(type_str, decoded.get('t', '0'), display_val))
            if len(self.all_rows) % 100 == 0: self.tree.yview_moveto(1.0)
        self.lbl_cnt.config(text=f"Packets: {len(self.all_rows)}")

    def toggle_connect(self):
        if hasattr(self, 'reader') and self.reader.running:
            self.reader.running = False
            self.btn_conn.config(text="Connect")
        else:
            p, b = self.cb_port.get(), self.cb_baud.get()
            if not p: return
            self.reader = SerialReader(p, int(b), lambda i, d: self.packet_queue.put((i, d)), lambda r: self.raw_bytes.extend(r))
            self.reader.start()
            self.btn_conn.config(text="Disconnect")

    def send_dump(self):
        if hasattr(self, 'reader') and self.reader.ser:
            self.reader.ser.write(b'PARSE\n')

    def refresh_ports(self):
        self.cb_port['values'] = [p.device for p in serial.tools.list_ports.comports()]
        if self.cb_port['values']: self.cb_port.current(0)

    def save_bin(self):
        path = filedialog.asksaveasfilename(defaultextension=".bin")
        if path:
            with open(path, "wb") as f: f.write(self.raw_bytes)
            messagebox.showinfo("Success", "Raw Binary Saved.")

    def save_xlsx(self):
        path = filedialog.asksaveasfilename(defaultextension=".xlsx")
        if not path or not self.all_rows: return
        wb = openpyxl.Workbook()
        sheets = {}
        for pkt_id, decoded in self.all_rows:
            sd = STRUCTS.get(pkt_id)
            if not sd: continue
            if pkt_id not in sheets:
                ws = wb.create_sheet(title=sd.name)
                ws.append(sd.fields); sheets[pkt_id] = ws
            sheets[pkt_id].append([decoded.get(f, '') for f in sd.fields])
        if 'Sheet' in wb.sheetnames: del wb['Sheet']
        wb.save(path)
        messagebox.showinfo("Success", "Excel Saved.")

if __name__ == "__main__":
    App().mainloop()