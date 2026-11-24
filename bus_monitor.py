"""
Moniteur des compteurs de bus ULPI (UART de l'Arty, ulpi_luna_acm.py).

Lit les lignes envoyées toutes les 0,5 s et affiche, au lieu des compteurs
bruts, ce qui s'est passé pendant chaque intervalle : nombre d'événements par
seconde, débit estimé, et alertes en clair.

    python -m pip install pyserial
    python bus_monitor.py COM7
"""

import re
import sys
import time

import serial

FIELDS = ["IN4", "OUT4", "PING4", "IN0", "NAK", "ACK", "NYET", "STALL", "D0", "D1", "HACK"]
PAT = re.compile(r"(\w+)=([0-9A-F]+)")
PACKET = 512


def parse(line):
    vals = dict(PAT.findall(line))
    if not all(f in vals for f in FIELDS):
        return None
    return {k: int(v, 16) for k, v in vals.items()}


def main(port):
    prev, t_prev = None, None
    # DTR/RTS maintenus inactifs : sur l'Arty, DTR du FTDI peut être relié au
    # reset du design (cavalier JP2 « CK_RST »). L'ouverture du port par
    # pyserial activerait DTR et redémarrerait le FPGA, donc la connexion USB.
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 2
    ser.dtr = False
    ser.rts = False
    ser.open()
    idle_reported = False
    with ser:
        print(f"Lecture de {port}… (Ctrl+C pour arrêter)\n")
        print(f"{'heure':8s} {'vit.':4s} {'SW1':3s} | {'IN4/s':>7s} {'données/s':>9s} {'≈ Mo/s IN':>9s} | "
              f"{'OUT4/s':>7s} {'≈ Mo/s OUT':>10s} {'PING/s':>7s} {'NYET/s':>7s} | {'NAK/s':>7s} {'IN0':>4s} | alertes")
        while True:
            line = ser.readline().decode(errors="replace").strip()
            cur = parse(line)
            if cur is None:
                continue
            t = time.perf_counter()
            if prev is not None:
                dt = t - t_prev
                d = {f: (cur[f] - prev[f]) & 0xFFFF for f in FIELDS}
                data = d["D0"] + d["D1"]
                alerts = []
                if d["STALL"] and not d["IN0"]:
                    alerts.append(f"{d['STALL']} STALL")
                if data and abs(data - d["HACK"]) > max(2, data // 100):
                    alerts.append(f"ACK manquants : {data - d['HACK']} paquets non acquittés")
                if d["IN0"]:
                    alerts.append(f"énumération / requêtes EP0 ({d['IN0']} IN0"
                                  + (f", {d['STALL']} STALL attendus" if d["STALL"] else "") + ")")
                idle = cur["TXV"] and not cur["TXR"] and not d["IN4"] and cur["SW1"]
                if idle and not idle_reported:
                    alerts.append("données prêtes, le PC ne lit pas (port COM fermé)")
                idle_reported = idle
                speed = {0: "HS", 1: "FS", 2: "LS"}.get(cur["SPD"], "?")
                print(f"{time.strftime('%H:%M:%S')} {speed:4s} {cur['SW1']:3d} | "
                      f"{d['IN4'] / dt:7.0f} {data / dt:9.0f} {data * PACKET / dt / 1e6:9.1f} | "
                      f"{d['OUT4'] / dt:7.0f} {d['ACK'] * PACKET / dt / 1e6:10.1f} "
                      f"{d['PING4'] / dt:7.0f} {d['NYET'] / dt:7.0f} | "
                      f"{d['NAK'] / dt:7.0f} {d['IN0']:4d} | " + "; ".join(alerts))
            prev, t_prev = cur, t


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage : python bus_monitor.py COMx")
    try:
        main(sys.argv[1])
    except KeyboardInterrupt:
        pass
