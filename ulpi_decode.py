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

table = list(csv.reader(open(sys.argv[1])))
header, rows = [h.split("[")[0] for h in table[0]], table[2:]

def col(*names):
    for n in names:
        if n in header:
            return header.index(n)
    return None

# Colonnes repérées par leur nom : ulpi_rxcmd.py (sondes ulpi_rxcmd/ulpi_rxdata)
# ou ulpi_luna_acm.py (RX CMD / octets déduits de DIR et NXT).
# (ulpi_regtest.py : noms dbg_* ; certaines sondes y apparaissent sous i / o$24 / o$26)
C_DIR, C_NXT = col("ulpi_dir", "dbg_dir", "i"), col("ulpi_nxt", "dbg_nxt")
C_STP, C_DOUT = col("ulpi_stp", "dbg_stp", "o$26"), col("ulpi_dout", "dbg_dout", "o$24")
C_DIN = col("ulpi_din", "dbg_din")
C_RXCMD, C_RXDATA = col("ulpi_rxcmd"), col("ulpi_rxdata")
C_SPEED, C_RESET, C_CONN = col("usb_speed"), col("usb_reset"), col("usb_connect")
if C_RXCMD is None and (C_DIR is None or C_NXT is None):
    sys.exit("Colonnes ulpi_dir/ulpi_nxt (ou ulpi_rxcmd/ulpi_rxdata) introuvables : " + ", ".join(header))

packets, cur, rx_active, last_cmd = [], None, False, None
events = []
tx_packets, tx_cur = [], None
state_events, last_state = [], None
dir_prev = 0
reg_pending = None
for r in rows:
    s = int(r[0])
    din = int(r[C_DIN], 16)
    if C_RXCMD is not None:
        rxcmd, rxdata = r[C_RXCMD] == "1", r[C_RXDATA] == "1"
    else:
        d, n = int(r[C_DIR], 16), int(r[C_NXT], 16)
        rxcmd, rxdata = bool(d and dir_prev and not n), bool(d and dir_prev and n)
        if rxcmd and reg_pending == "rd":
            rxcmd, reg_pending = False, None      # valeur de registre lue, pas un RX CMD
        # côté émission (FPGA -> PHY) : TX CMD puis octets acceptés par NXT ;
        # les commandes registre (10 = écriture, 11 = lecture) sont écartées
        if C_DOUT is not None:
            dout, stp = int(r[C_DOUT], 16), int(r[C_STP], 16)
            if not d and n and tx_cur is None:
                if reg_pending == "wr":
                    reg_pending = None            # octet de donnée de l'écriture registre
                elif (dout >> 6) == 1:
                    tx_cur = {"start": s, "bytes": [0x0F & dout | ((~dout & 0x0F) << 4)]}
                elif (dout >> 6) == 2:
                    reg_pending = "wr"
                elif (dout >> 6) == 3:
                    reg_pending = "rd"
            elif tx_cur is not None and not d and n:
                tx_cur["bytes"].append(dout)
            if tx_cur is not None and (stp or d):
                tx_packets.append(tx_cur); tx_cur = None
        dir_prev = d
    st = tuple(r[c] for c in (C_CONN, C_RESET, C_SPEED) if c is not None)
    if st != last_state:
        state_events.append((s, st)); last_state = st
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

if state_events and C_CONN is not None:
    print()
    print("État LUNA (échantillon : connect reset speed[0 HS, 1 FS])")
    for s, st in state_events[:20]:
        print(f"  {s:5d} : " + " ".join(st))
if tx_packets:
    print()
    print(f"{len(tx_packets)} paquet(s) émis par le FPGA :")
    for p in tx_packets[:40]:
        bs = p["bytes"]
        print(f"  éch. {p['start']:5d} : {PIDS.get(bs[0], '?'):6s} " + " ".join(f"{b:02X}" for b in bs[:24])
              + (" …" if len(bs) > 24 else ""))
