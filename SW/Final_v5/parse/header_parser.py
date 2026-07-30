import os
import re
import struct


# C type -> Python struct format mapping.
TYPE_MAP = {
    "uint8_t": "B",
    "int8_t": "b",
    "uint16_t": "H",
    "int16_t": "h",
    "uint32_t": "I",
    "int32_t": "i",
    "float": "f",
    "double": "d",
}

# Friendly names for the packed firmware arrays in state_pkt.
ARRAY_FIELD_NAMES = {
    "p": ["pN", "pE", "pD"],
    "v": ["vN", "vE", "vD"],
    "q": ["qw", "qx", "qy", "qz"],
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


def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//.*", "", text)


def _array_names(base_name, count):
    mapped = ARRAY_FIELD_NAMES.get(base_name)
    if mapped and len(mapped) == count:
        return mapped
    return [f"{base_name}{i}" for i in range(count)]


def _parse_declarations(body):
    """Return (ctype, field_name) entries from a C struct body.

    Handles the firmware style used by sensor_data.h:
    - PacketHeader header;
    - int16_t gx, gy, gz;
    - float p[3];
    - uint8_t SYNC_BYTE = 0xAA;
    """
    entries = []
    for line in _strip_comments(body).splitlines():
        line = line.strip()
        if not line:
            continue

        if re.search(r"\bPacketHeader\b", line):
            entries.append(("PacketHeader", "header"))
            continue

        match = re.match(r"(\w+_t|float|double)\s+([^;]+);", line)
        if not match:
            continue

        ctype, declarators = match.groups()
        if ctype not in TYPE_MAP:
            continue

        for decl in declarators.split(","):
            decl = decl.strip().split("=")[0].strip()
            arr = re.match(r"(\w+)\s*\[\s*(\d+)\s*\]$", decl)
            if arr:
                base_name = arr.group(1)
                count = int(arr.group(2))
                for field_name in _array_names(base_name, count):
                    entries.append((ctype, field_name))
                continue

            name = re.match(r"(\w+)", decl)
            if name:
                entries.append((ctype, name.group(1)))

    return entries


def parse_header(header_path):
    if not os.path.exists(header_path):
        print(f"Error: {header_path} not found.")
        return {}

    with open(header_path, "r", encoding="utf-8") as f:
        text = f.read()

    id_map = {
        m.group(1).lower(): int(m.group(2))
        for m in re.finditer(r"#define\s+ID_(\w+)\s+(\d+)", text)
    }

    struct_pattern = re.compile(r"struct\s+(\w+)\s*\{([^}]+)\}", re.DOTALL)
    raw_structs = {m.group(1): m.group(2) for m in struct_pattern.finditer(text)}

    header_fmt = []
    if "PacketHeader" in raw_structs:
        for ctype, _ in _parse_declarations(raw_structs["PacketHeader"]):
            if ctype in TYPE_MAP:
                header_fmt.append(TYPE_MAP[ctype])
    header_count = len(header_fmt)

    structs = {}
    for sname, body in raw_structs.items():
        if sname == "PacketHeader":
            continue

        sname_lower = sname.lower()
        pkt_id = id_map.get(sname_lower)
        if pkt_id is None:
            stripped = sname_lower.replace("_pkt", "").replace("_packet", "")
            pkt_id = id_map.get(stripped)
        if pkt_id is None:
            for key, val in id_map.items():
                if key in sname_lower:
                    pkt_id = val
                    break
        if pkt_id is None:
            continue

        fields = []
        fmt_codes = ["<"]
        for ctype, field_name in _parse_declarations(body):
            if ctype == "PacketHeader":
                fmt_codes.extend(header_fmt)
                continue
            fmt_codes.append(TYPE_MAP[ctype])
            fields.append(field_name)

        fmt_str = "".join(fmt_codes)
        structs[pkt_id] = StructDef(
            name=sname,
            pkt_id=pkt_id,
            fields=fields,
            fmt=fmt_str,
            size=struct.calcsize(fmt_str),
            h_count=header_count,
        )

    return structs
