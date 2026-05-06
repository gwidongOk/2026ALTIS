import re, struct, os

# C 타입 -> Python struct 포맷 코드 매핑
TYPE_MAP = {
    'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h',
    'uint32_t': 'I', 'int32_t': 'i', 'float': 'f', 'double': 'd',
}

class StructDef:
    def __init__(self, name, pkt_id, fields, fmt, size, h_count):
        self.name = name
        self.pkt_id = pkt_id
        self.fields = fields
        self.fmt = fmt
        self.size = size
        self.header_field_count = h_count

    def decode(self, raw):
        values = struct.unpack(self.fmt, raw[:self.size])
        return dict(zip(self.fields, values[self.header_field_count:]))

def parse_header(header_path):
    if not os.path.exists(header_path):
        print(f"Error: {header_path} not found.")
        return {}
        
    with open(header_path, 'r', encoding='utf-8') as f:
        text = f.read()

    # 1. #define ID_... 추출
    id_map = {m.group(1).lower(): int(m.group(2)) 
              for m in re.finditer(r'#define\s+ID_(\w+)\s+(\d+)', text)}

    # 2. 구조체 본문 추출
    struct_pattern = re.compile(r'struct\s+(\w+)\s*\{([^}]+)\}', re.DOTALL)
    raw_structs = {m.group(1): m.group(2) for m in struct_pattern.finditer(text)}

    # 3. PacketHeader 분석 (공통 헤더 필드 개수 계산)
    h_fmt = []
    h_count = 0
    if 'PacketHeader' in raw_structs:
        for line in raw_structs['PacketHeader'].split('\n'):
            parts = line.strip().split()
            if len(parts) >= 2 and parts[0] in TYPE_MAP:
                h_fmt.append(TYPE_MAP[parts[0]])
        h_count = len(h_fmt)

    # 4. 각 센서 패킷 정의 생성
    structs = {}
    for sname, body in raw_structs.items():
        if sname == 'PacketHeader': continue

        # 구조체 이름에서 ID 매칭: "baro_pkt" → "baro", "imu_pkt" → "imu"
        sname_lower = sname.lower()
        pkt_id = id_map.get(sname_lower)
        if pkt_id is None:
            # _pkt 접미사 제거 후 재시도
            stripped = sname_lower.replace('_pkt', '').replace('_packet', '')
            pkt_id = id_map.get(stripped)
        if pkt_id is None:
            # ID 키가 구조체 이름에 포함되는지 확인
            for key, val in id_map.items():
                if key in sname_lower:
                    pkt_id = val
                    break
        if pkt_id is None: continue

        fields, fmt_codes = [], ['<']
        for line in body.split('\n'):
            line = line.strip()
            if 'PacketHeader' in line:
                fmt_codes.extend(h_fmt)
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[0] in TYPE_MAP:
                fmt_codes.append(TYPE_MAP[parts[0]])
                fields.append(parts[1].rstrip(';'))

        fmt_str = ''.join(fmt_codes)
        structs[pkt_id] = StructDef(sname, pkt_id, fields, fmt_str, struct.calcsize(fmt_str), h_count)
    
    return structs