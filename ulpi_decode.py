"""Décode une capture ILA ULPI exportée en CSV (write_hw_ila_data -csv_file)."""
import csv, sys

PIDS = {0xE1:"OUT",0x69:"IN",0xA5:"SOF",0x2D:"SETUP",0xC3:"DATA0",0x4B:"DATA1",
        0x87:"DATA2",0x0F:"MDATA",0xD2:"ACK",0x5A:"NAK",0x1E:"STALL",0x96:"NYET",
        0x3C:"PRE/ERR",0x78:"SPLIT",0xB4:"PING"}
LS = {0:"SE0",1:"J",2:"K",3:"SE1"}
EV = {0:"-",1:"RxActive",2:"HostDisc",3:"RxError"}

def crc5_ok(b1, b2):
    v = b1 | (b2 << 8)          # 11 bits de données + 5 bits de CRC
    data, crc = v & 0x7FF, (v >> 11) & 0x1F
    c = 0x1F
    for i in range(11):
        bit = (data >> i) & 1
        if (c & 1) ^ bit: c = (c >> 1) ^ 0x14
        else: c >>= 1
    return (c ^ 0x1F) == crc

def crc16_ok(payload):
    c = 0xFFFF
    for b in payload:
        for i in range(8):
            bit = (b >> i) & 1
            if (c & 1) ^ bit: c = (c >> 1) ^ 0xA001
            else: c >>= 1
    return c == 0xB001          # résidu attendu en incluant le CRC reçu

rows = list(csv.reader(open(sys.argv[1])))[2:]
packets, cur, rx_active, last_cmd = [], None, False, None
events = []
for r in rows:
    s = int(r[0]); trig = r[2]
    rxcmd, rxdata = r[11] == "1", r[12] == "1"
    din = int(r[8], 16)
    if rxcmd:
        ev = (din >> 4) & 3
        if din != last_cmd:
            events.append((s, din))
        last_cmd = din
        if ev == 1 and not rx_active:
            rx_active = True; cur = {"start": s, "bytes": []}
        elif ev != 1 and rx_active:
            rx_active = False; cur["end"] = s; packets.append(cur); cur = None
    if rxdata:
        if cur is None:
            cur = {"start": s, "bytes": []}; rx_active = True
        cur["bytes"].append((s, din))
if cur: cur["end"] = None; packets.append(cur)

print("Changements de RX CMD (échantillon : valeur  LineState  Vbus  RxEvent)")
for s, v in events[:12]:
    print(f"  {s:5d} : {v:02X}  {LS[v&3]:3s}  VB={(v>>2)&3}  {EV[(v>>4)&3]}")
if len(events) > 12: print(f"  ... ({len(events)} changements au total)")
print()
print(f"{len(packets)} paquet(s) :")
for p in packets:
    bs = [b for _, b in p["bytes"]]
    if not bs:
        continue
    pid = bs[0]
    name = PIDS.get(pid, "?") if (pid & 0xF) == (~pid >> 4) & 0xF else "PID invalide"
    line = f"  éch. {p['start']:5d}-{p['end'] if p['end'] is not None else 'fin'} : {name:6s} " + " ".join(f"{b:02X}" for b in bs)
    if name in ("SETUP", "IN", "OUT", "SOF") and len(bs) == 3:
        v = bs[1] | (bs[2] << 8)
        extra = f"   frame={v & 0x7FF}" if name == "SOF" else f"   adresse={v & 0x7F} endpoint={(v >> 7) & 0xF}"
        line += extra + ("  CRC5 OK" if crc5_ok(bs[1], bs[2]) else "  CRC5 FAUX")
    elif name.startswith("DATA"):
        line += "  CRC16 OK" if crc16_ok(bs[1:]) else "  CRC16 FAUX"
        d = bs[1:-2]
        if name == "DATA0" and len(d) == 8:
            bm, br, wv, wi, wl = d[0], d[1], d[2] | d[3] << 8, d[4] | d[5] << 8, d[6] | d[7] << 8
            req = {6: "GET_DESCRIPTOR", 5: "SET_ADDRESS", 9: "SET_CONFIGURATION", 0: "GET_STATUS"}.get(br, f"bRequest={br}")
            line += f"\n      requête : bmRequestType={bm:02X} {req} wValue={wv:04X} wIndex={wi:04X} wLength={wl}"
            if br == 6:
                line += f"  (descripteur type {wv >> 8}, index {wv & 0xFF})"
    print(line)