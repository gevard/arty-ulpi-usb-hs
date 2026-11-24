"""
ULPI étape 1 — lecture des registres d'identification du USB3300 et test du
registre Scratch, sur Arty A7-35T + module Waveshare USB3300.

Câblage (Pmod JB = pmod 1, Pmod JC = pmod 2) :
    JB1 E15 STP    JB2 E16 NXT    JB3 D15 CLK (MRCC, côté P)   JB4 C15 DIR
    JB10 J15 RESET (actif haut)
    JC1..JC4, JC7..JC10 : DATA0..DATA7

Résultat :
    LD4 : battement (100 MHz, prouve que le FPGA est configuré)
    LD5 : MMCM verrouillé sur le 60 MHz du PHY
    LD6 : Vendor ID = 0x0424 et Product ID = 0x0004
    LD7 : écriture puis relecture du registre Scratch OK
    UART (port COM de l'Arty, 115200 8N1), toutes les 0,5 s :
        VID=0424 PID=0004 SCR=5A OK

Construction (Vivado dans le PATH) :
    python ulpi_regtest.py            -> build/ulpi_regtest.bit
    python ulpi_regtest.py --program  -> construit et programme via Vivado
"""

import argparse
import os
import subprocess

from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer, ResetSynchronizer
from amaranth.build import Resource, Subsignal, Pins, Attrs, Clock
from amaranth_boards.arty_a7 import ArtyA7_35Platform


SYS_HZ = 100_000_000
ULPI_HZ = 60_000_000
BAUD = 115_200

USB3300_VID = 0x0424
USB3300_PID = 0x0004   # lu sur le module (0x0007 = USB3320, 0x0006 = USB331x)
SCRATCH_ADDR = 0x16
SCRATCH_VAL = 0x5A

ULPI_RESOURCE = [
    Resource("ulpi", 0,
        Subsignal("stp",  Pins("1",  dir="o", conn=("pmod", 1))),
        Subsignal("nxt",  Pins("2",  dir="i", conn=("pmod", 1))),
        Subsignal("clk",  Pins("3",  dir="i", conn=("pmod", 1)), Clock(ULPI_HZ)),
        Subsignal("dir",  Pins("4",  dir="i", conn=("pmod", 1))),
        Subsignal("rst",  Pins("10", dir="o", conn=("pmod", 1))),
        Subsignal("data", Pins("1 2 3 4 7 8 9 10", dir="io", conn=("pmod", 2))),
        Attrs(IOSTANDARD="LVCMOS33", SLEW="FAST", DRIVE="8"),
    ),
]


def hexchar(nibble):
    """Quartet -> caractère ASCII hexadécimal (majuscules)."""
    return Mux(nibble < 10, nibble + ord("0"), nibble + (ord("A") - 10))


class UartTx(Elaboratable):
    """Émetteur UART 8N1 minimal (domaine sync)."""

    def __init__(self, divisor):
        self.divisor = divisor
        self.data = Signal(8)
        self.start = Signal()
        self.busy = Signal()
        self.tx = Signal(init=1)

    def elaborate(self, platform):
        m = Module()
        cnt = Signal(range(self.divisor))
        shreg = Signal(11)
        bits = Signal(range(12))
        m.d.comb += self.busy.eq(bits != 0)
        with m.If(bits == 0):
            with m.If(self.start):
                # start, 8 bits LSB d'abord, stop, + 1 bit de repos pour un stop complet
                m.d.sync += [shreg.eq(Cat(C(0, 1), self.data, C(0b11, 2))),
                             bits.eq(11), cnt.eq(0)]
        with m.Else():
            with m.If(cnt == 0):
                m.d.sync += [self.tx.eq(shreg[0]), shreg.eq(shreg >> 1),
                             bits.eq(bits - 1), cnt.eq(self.divisor - 1)]
            with m.Else():
                m.d.sync += cnt.eq(cnt - 1)
        return m


class Top(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        # ------------------------------------------------------------------
        # E/S brutes (Amaranth 0.5 : dir="-" puis buffers explicites)
        # ------------------------------------------------------------------
        ulpi = platform.request("ulpi", 0, dir="-")
        m.submodules.clk_buf = clk_buf = io.Buffer("i", ulpi.clk)
        m.submodules.dir_buf = dir_buf = io.Buffer("i", ulpi.dir)
        m.submodules.nxt_buf = nxt_buf = io.Buffer("i", ulpi.nxt)
        m.submodules.stp_buf = stp_buf = io.Buffer("o", ulpi.stp)
        m.submodules.rst_buf = rst_buf = io.Buffer("o", ulpi.rst)
        m.submodules.dat_buf = dat_buf = io.Buffer("io", ulpi.data)

        leds = [platform.request("led", i).o for i in range(4)]
        uart = platform.request("uart", 0)

        # ------------------------------------------------------------------
        # Séquence de démarrage (domaine sync = 100 MHz de la carte)
        #   0..1 ms  : RESET PHY actif
        #   0..5 ms  : MMCM en reset (le PHY démarre sa PLL et sort CLKOUT)
        #   si pas verrouillé après 50 ms : on recommence
        # ------------------------------------------------------------------
        T_PHY_RST = SYS_HZ // 1000
        T_MMCM_RST = 5 * SYS_HZ // 1000
        T_RETRY = 50 * SYS_HZ // 1000
        boot = Signal(range(T_RETRY + 1))
        phy_rst = Signal(init=1)
        mmcm_rst = Signal(init=1)
        locked = Signal()
        locked_sys = Signal()
        m.submodules.lock_sync = FFSynchronizer(locked, locked_sys)

        with m.If(boot < T_RETRY):
            m.d.sync += boot.eq(boot + 1)
        with m.Elif(~locked_sys):
            m.d.sync += boot.eq(0)  # pas d'horloge PHY : nouvelle tentative
        m.d.sync += [phy_rst.eq(boot < T_PHY_RST), mmcm_rst.eq(boot < T_MMCM_RST)]
        m.d.comb += rst_buf.o.eq(phy_rst)

        # ------------------------------------------------------------------
        # Horloge ULPI : CLKOUT 60 MHz du PHY -> MMCM en compensation (zéro
        # délai) -> BUFG. L'horloge interne est alignée sur la broche, ce qui
        # garde les marges de setup/hold de l'ULPI.
        # VCO = 60 MHz x 16 = 960 MHz, sortie /16 = 60 MHz.
        # ------------------------------------------------------------------
        clkfb = Signal()
        clkfb_bufg = Signal()
        clk0 = Signal()
        m.domains.ulpi = cd_ulpi = ClockDomain("ulpi")
        m.submodules.mmcm = Instance("MMCME2_BASE",
            p_CLKIN1_PERIOD=1e9 / ULPI_HZ,
            p_BANDWIDTH="OPTIMIZED",
            p_DIVCLK_DIVIDE=1,
            p_CLKFBOUT_MULT_F=16.0,
            p_CLKOUT0_DIVIDE_F=16.0,
            i_CLKIN1=clk_buf.i,
            i_CLKFBIN=clkfb_bufg,
            o_CLKFBOUT=clkfb,
            o_CLKOUT0=clk0,
            o_LOCKED=locked,
            i_RST=mmcm_rst,
            i_PWRDWN=0,
        )
        m.submodules.bufg_fb = Instance("BUFG", i_I=clkfb, o_O=clkfb_bufg)
        m.submodules.bufg_ulpi = Instance("BUFG", i_I=clk0, o_O=cd_ulpi.clk)
        m.submodules.ulpi_rst = ResetSynchronizer(~locked, domain="ulpi")

        # ------------------------------------------------------------------
        # Bus ULPI
        # Règle fondamentale : le lien ne pilote DATA que quand DIR = 0.
        # OE est donc dérivé combinatoirement de DIR (cycle de retournement).
        # Au repos, le lien pilote 0x00 (NOOP).
        # ------------------------------------------------------------------
        dir_ = dir_buf.i
        nxt = nxt_buf.i
        data_o = Signal(8)
        stp = Signal()
        m.d.comb += [dat_buf.oe.eq(~dir_), dat_buf.o.eq(data_o), stp_buf.o.eq(stp)]

        # Programme de test : (écriture?, adresse)
        steps = [(1, SCRATCH_ADDR),   # écrire 0x5A dans Scratch
                 (0, 0x00), (0, 0x01),  # Vendor ID bas / haut
                 (0, 0x02), (0, 0x03),  # Product ID bas / haut
                 (0, SCRATCH_ADDR)]     # relire Scratch
        step = Signal(range(len(steps)))
        step_wr = Array(C(w, 1) for w, _ in steps)[step]
        step_addr = Array(C(a, 6) for _, a in steps)[step]
        res = Array(Signal(8, name=f"res{i}") for i in range(len(steps)))

        timer = Signal(range(ULPI_HZ // 2 + 1), init=ULPI_HZ // 1000)  # 1 ms au départ
        timeout = Signal(10)
        fail = Signal()
        report = Signal()

        with m.FSM(domain="ulpi"):
            with m.State("WAIT"):
                with m.If(timer != 0):
                    m.d.ulpi += timer.eq(timer - 1)
                with m.Elif(~dir_):
                    m.d.ulpi += [step.eq(0), fail.eq(0), timeout.eq(0)]
                    m.next = "CMD"

            # TX CMD : 0b10aaaaaa = RegWrite, 0b11aaaaaa = RegRead.
            # Maintenu jusqu'à NXT. Si DIR monte avant, le PHY prend le bus
            # (RX CMD) : transaction abandonnée, on recommence.
            with m.State("CMD"):
                # bit 7 = 1 (commande registre), bit 6 = 1 lecture / 0 écriture
                m.d.comb += data_o.eq(Cat(step_addr, step_wr == 0, C(1, 1)))
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "ABORT"
                with m.Elif(nxt):
                    m.d.ulpi += timeout.eq(0)
                    with m.If(step_wr):
                        m.next = "WR_DATA"
                    with m.Else():
                        m.next = "RD_TURN"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            # Écriture : donnée maintenue jusqu'à NXT, puis STP un cycle.
            with m.State("WR_DATA"):
                m.d.comb += data_o.eq(SCRATCH_VAL)
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "ABORT"
                with m.Elif(nxt):
                    m.next = "WR_STP"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            with m.State("WR_STP"):
                m.d.comb += stp.eq(1)
                m.next = "NEXT"

            # Lecture : après NXT, le PHY monte DIR (cycle de retournement),
            # puis présente la donnée au cycle suivant, puis redescend DIR.
            with m.State("RD_TURN"):
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "RD_DATA"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            with m.State("RD_DATA"):
                m.d.ulpi += res[step].eq(dat_buf.i)
                with m.If(nxt):
                    # NXT avec DIR = 1 ici = RX CMD prioritaire, lecture perdue
                    m.next = "ABORT"
                with m.Else():
                    m.next = "RD_END"

            with m.State("RD_END"):
                with m.If(~dir_):
                    m.next = "NEXT"

            with m.State("ABORT"):
                with m.If(~dir_):
                    m.d.ulpi += timeout.eq(0)
                    m.next = "CMD"

            with m.State("NEXT"):
                with m.If(step == len(steps) - 1):
                    m.d.ulpi += timer.eq(ULPI_HZ // 2)
                    m.d.comb += report.eq(1)
                    m.next = "WAIT"
                with m.Else():
                    m.d.ulpi += step.eq(step + 1)
                    m.next = "CMD"

        vid = Cat(res[1], res[2])
        pid = Cat(res[3], res[4])
        scr = res[5]
        id_ok = Signal()
        scr_ok = Signal()
        with m.If(report):
            m.d.ulpi += [id_ok.eq(~fail & (vid == USB3300_VID) & (pid == USB3300_PID)),
                         scr_ok.eq(~fail & (scr == SCRATCH_VAL))]

        # ------------------------------------------------------------------
        # Rapport UART (domaine ulpi, 60 MHz / 115200 -> diviseur 521)
        # ------------------------------------------------------------------
        m.submodules.tx = tx = DomainRenamer("ulpi")(UartTx(round(ULPI_HZ / BAUD)))
        ok = Signal()
        m.d.comb += ok.eq(~fail & (vid == USB3300_VID) & (pid == USB3300_PID)
                          & (scr == SCRATCH_VAL))

        def s(text):
            return [C(ord(c), 8) for c in text]

        chars = (s("VID=") + [hexchar(vid[i:i + 4]) for i in (12, 8, 4, 0)]
                 + s(" PID=") + [hexchar(pid[i:i + 4]) for i in (12, 8, 4, 0)]
                 + s(" SCR=") + [hexchar(scr[i:i + 4]) for i in (4, 0)]
                 + s(" ")
                 + [Mux(ok, ord("O"), ord("E")), Mux(ok, ord("K"), ord("R")),
                    Mux(ok, ord(" "), ord("R"))]
                 + s("\r\n"))
        idx = Signal(range(len(chars) + 1))
        ch = Array(chars)[idx]

        with m.FSM(domain="ulpi"):
            with m.State("IDLE"):
                with m.If(report):
                    m.d.ulpi += idx.eq(0)
                    m.next = "CHAR"
            with m.State("CHAR"):
                m.d.comb += [tx.data.eq(ch), tx.start.eq(1)]
                with m.If(~tx.busy):
                    m.d.ulpi += idx.eq(idx + 1)
                    m.next = "GAP"
            with m.State("GAP"):
                m.next = "BUSY"
            with m.State("BUSY"):
                with m.If(~tx.busy):
                    with m.If(idx == len(chars)):
                        m.next = "IDLE"
                    with m.Else():
                        m.next = "CHAR"

        # ------------------------------------------------------------------
        # LED et UART
        # ------------------------------------------------------------------
        beat = Signal(27)
        m.d.sync += beat.eq(beat + 1)
        m.d.comb += [
            leds[0].eq(beat[-1]),
            leds[1].eq(locked_sys),
            leds[2].eq(id_ok),
            leds[3].eq(scr_ok),
            uart.tx.o.eq(tx.tx),
        ]
        return m


PROGRAM_TCL = """open_hw_manager
connect_hw_server
open_hw_target
set dev [lindex [get_hw_devices xc7a35t*] 0]
current_hw_device $dev
set_property PROGRAM.FILE {{{bit}}} $dev
program_hw_devices $dev
close_hw_manager
"""


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--program", action="store_true", help="programmer la carte après la construction")
    ap.add_argument("--no-build", action="store_true", help="générer seulement les fichiers (sans Vivado)")
    args = ap.parse_args()

    platform = ArtyA7_35Platform()
    platform.add_resources(ULPI_RESOURCE)
    if args.no_build:
        platform.prepare(Top(), name="ulpi_regtest").extract("build")
    else:
        platform.build(Top(), name="ulpi_regtest", build_dir="build")

    if args.program:
        bit = os.path.abspath(os.path.join("build", "ulpi_regtest.bit")).replace("\\", "/")
        tcl = os.path.join("build", "program.tcl")
        with open(tcl, "w") as f:
            f.write(PROGRAM_TCL.format(bit=bit))
        vivado = "vivado.bat" if os.name == "nt" else "vivado"
        subprocess.run([vivado, "-mode", "batch", "-nojournal", "-nolog", "-source", tcl], check=True)