"""
Essais côté PC du périphérique CDC-ACM LUNA (ulpi_luna_acm.py).

    python -m pip install pyserial
    python test_acm.py echo      # SW0 = 1, SW1 = 0 : test d'écho
    python test_acm.py read      # SW0 = 1, SW1 = 1 : débit FPGA -> PC
    python test_acm.py write     # SW0 = 1, SW1 = 1 : débit PC -> FPGA

Le port est trouvé automatiquement par son VID/PID (1209:0001) ; sinon --port COMx.
"""

import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports

VID, PID = 0x1209, 0x0001


def find_port():
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    sys.exit("Périphérique 1209:0001 introuvable (SW0 en haut ? énumération réussie ?)")


def drain(ser, quiet=0.2):
    """Jette les octets en attente (reste du mode débit SW1 = 1, encore dans
    le tampon du PC ou dans l'endpoint IN du FPGA) jusqu'à `quiet` s de silence."""
    ser.reset_input_buffer()
    timeout, ser.timeout = ser.timeout, quiet
    dropped = 0
    while chunk := ser.read(1 << 16):
        dropped += len(chunk)
        if dropped > 1 << 20:
            sys.exit("Le FPGA envoie en continu : SW1 est encore à 1 (mettre SW1 = 0 pour l'écho)")
    ser.timeout = timeout
    if dropped:
        print(f"({dropped} octets en attente ignorés)")


def echo(ser, rounds=200):
    drain(ser)
    total, t0 = 0, time.perf_counter()
    for n in range(rounds):
        size = 1 + (n * 37) % 4096
        data = os.urandom(size)
        try:
            ser.write(data)
        except serial.SerialTimeoutException:
            sys.exit(f"ERREUR au tour {n} : écriture refusée (OUT jamais acquitté)")
        back = ser.read(size)
        if back != data:
            first = next((i for i, (a, b) in enumerate(zip(data, back)) if a != b), len(back))
            sys.exit(f"ERREUR au tour {n} : {len(back)}/{size} octets reçus, premier écart à l'octet {first}")
        total += size
    dt = time.perf_counter() - t0
    print(f"Écho OK : {rounds} échanges, {total} octets, {total / dt / 1e6:.2f} Mo/s aller-retour")


def read_rate(ser, seconds=5.0):
    ser.reset_input_buffer()
    total, t0, expected, errors = 0, time.perf_counter(), None, 0
    while time.perf_counter() - t0 < seconds:
        chunk = ser.read(1 << 16)
        if not chunk:
            continue
        if expected is None:
            expected = chunk[0]
        # vérifie la continuité du compteur 8 bits
        for b in chunk[:64]:
            if b != expected:
                errors += 1
                expected = b
            expected = (expected + 1) & 0xFF
        expected = (chunk[-1] + 1) & 0xFF
        total += len(chunk)
    dt = time.perf_counter() - t0
    print(f"Lecture : {total / dt / 1e6:.2f} Mo/s ({total} octets en {dt:.1f} s), "
          f"discontinuités du compteur (échantillonnées) : {errors}")


def write_rate(ser, seconds=5.0):
    block = bytes(range(256)) * 256          # 64 kio
    total, t0 = 0, time.perf_counter()
    try:
        while time.perf_counter() - t0 < seconds:
            total += ser.write(block)
        ser.flush()
    except serial.SerialTimeoutException:
        print(f"Écriture bloquée : le périphérique n'accepte plus de données ({total} octets envoyés)")
    dt = time.perf_counter() - t0
    print(f"Écriture : {total / dt / 1e6:.2f} Mo/s ({total} octets en {dt:.1f} s)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["echo", "read", "write"])
    ap.add_argument("--port")
    args = ap.parse_args()
    port = args.port or find_port()
    print(f"Port : {port}")
    with serial.Serial(port, timeout=1, write_timeout=2) as ser:
        {"echo": echo, "read": read_rate, "write": write_rate}[args.test](ser)