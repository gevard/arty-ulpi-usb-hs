"""
ULPI étape 2 — RX CMD, LineState, VBUS et pull-up D+, sur Arty A7-35T + USB3300.

Reprend le test de registres de l'étape 1 et ajoute :
  * la détection des RX CMD : le PHY prend le bus de lui-même (DIR = 1,
    NXT = 0) pour signaler un changement d'état de la ligne USB ;
  * le comptage des octets de paquets reçus (DIR = 1 et NXT = 1) ;
  * l'interrupteur SW0 : écrit OTG Control (0x0A) et Function Control (0x04)
        SW0 = 0 -> OTG 0x06, FC 0x41 : pull-down D+/D- actifs, pas de pull-up
        SW0 = 1 -> OTG 0x00, FC 0x45 : pull-down coupés, pull-up 1,5 kΩ sur D+
    (configuration « périphérique FS » de la spec ULPI : pull-down D+/D- à 0)
    Avec le câble branché sur un PC et SW0 = 1, le PC détecte un périphérique
    Full-Speed, le réinitialise et lui envoie des requêtes (sans réponse :
    Windows finira par signaler un « périphérique USB inconnu »).

Câblage (Pmod JB = pmod 1, Pmod JC = pmod 2) :
    JB1 E15 STP    JB2 E16 NXT    JB3 D15 CLK (MRCC, côté P)   JB4 C15 DIR
    JB10 J15 RESET (actif haut)
    JC1..JC4, JC7..JC10 : DATA0..DATA7

LED :
    LD4 : battement 100 MHz
    LD5 : MMCM verrouillé sur le 60 MHz du PHY
    LD6 : test de registres OK (ID, Scratch, relecture de Function Control)
    LD7 : VBUS valide (d'après le dernier RX CMD)

UART (115200 8N1) :
    toutes les 0,5 s :  VID=0424 PID=0004 SCR=5A FC=45 RXC=0003 PKT=0000 OK
        RXC = nombre de RX CMD reçus, PKT = nombre d'octets de paquets reçus
    à chaque nouveau RX CMD (si l'UART est libre) :
        RXCMD=4D LS=J VB=3 EV=0
        LS = LineState (0 = SE0, J, K, 1 = SE1)
        VB = VbusState (0 < SessEnd, 1 SessEnd..SessVld, 2 SessVld..VbusVld, 3 VBUS valide)
        EV = RxEvent (0 rien, 1 RxActive, 2 HostDisconnect, 3 RxError)

ILA (IP Vivado, horloge ULPI, 4096 échantillons, 5 sondes ; licence BASIC).
Toutes les sondes passent par un registre : elles sont retardées d'un cycle,
de la même façon pour toutes, et portent des noms lisibles :
    ulpi_dir, ulpi_nxt, ulpi_stp, ulpi_oe, ulpi_dout[8], ulpi_din[8],
    ulpi_fsm[4] (0 WAIT, 1 CMD, 2 WR_DATA, 3 WR_STP, 4 RD_TURN, 5 RD_DATA,
                  6 RD_END, 7 ABORT, 8 NEXT),
    ulpi_step[4] (0 écr. Scratch, 1 écr. OTG Ctrl, 2 écr. FuncCtrl, 3-4 VID,
                  5-6 PID, 7 lect. Scratch, 8 lect. FuncCtrl),
    ulpi_rxcmd  (1 = l'octet présent sur ulpi_din est un RX CMD),
    ulpi_rxdata (1 = l'octet présent sur ulpi_din est un octet de paquet)

Construction (Vivado dans le PATH, variable VIVADO = chemin de vivado.bat) :
    python ulpi_rxcmd.py --program
    python ulpi_rxcmd.py --no-ila --program
    python ulpi_rxcmd.py --program-only   -> programme build/ulpi_rxcmd.bit (+ .ltx)
"""

import argparse
import os
import subprocess

from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer, ResetSynchronizer
from amaranth.build import Resource, Subsignal, Pins, Attrs, Clock
from amaranth_boards.arty_a7 import ArtyA7_35Platform


NAME = "ulpi_rxcmd"
SYS_HZ = 100_000_000
ULPI_HZ = 60_000_000
BAUD = 115_200

USB3300_VID = 0x0424
USB3300_PID = 0x0009   # USB3300 (0x0006 = USB331x, 0x0007 = USB3320, 0x0009 = USB3343)
REG_FUNC_CTRL = 0x04
REG_OTG_CTRL = 0x0A
REG_SCRATCH = 0x16
SCRATCH_VAL = 0x5A
FC_NO_PULLUP = 0x41    # SuspendM=1, OpMode=00, TermSelect=0, XcvrSelect=01 (FS)
FC_PULLUP = 0x45       # idem avec TermSelect=1
OTG_HOST = 0x06        # valeur au reset : DpPulldown=1, DmPulldown=1 (pull-down 15 kΩ)
OTG_DEVICE = 0x00      # périphérique : pull-down désactivés

ULPI_RESOURCE = [
    Resource("ulpi", 0,
        Subsignal("stp",  Pins("1",  dir="o", conn=("pmod", 1))),
        Subsignal("nxt",  Pins("2",  dir="i", conn=("pmod", 1))),
        Subsignal("clk",  Pins("3",  dir="i", conn=("pmod", 1)), Clock(ULPI_HZ)),
        Subsignal("dir",  Pins("4",  dir="i", conn=("pmod", 1))),
        Subsignal("rst",  Pins("10", dir="o", conn=("pmod", 1))),
        Subsignal("data", Pins("1 2 3 4 7 8 9 10", dir="io", conn=("pmod", 2))),
        Attrs(IOSTANDARD="LVCMOS33"),
    ),
]


def hexchar(nibble):
    return Mux(nibble < 10, nibble + ord("0"), nibble + (ord("A") - 10))


def hexchars(value, digits):
    return [hexchar(value[4 * i:4 * i + 4]) for i in reversed(range(digits))]


def text(s):
    return [C(ord(c), 8) for c in s]


class UartTx(Elaboratable):
    """Émetteur UART 8N1 minimal."""

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
    def __init__(self, with_ila=True):
        self.with_ila = with_ila

    def elaborate(self, platform):
        m = Module()

        # ------------------------------------------------------------------
        # E/S
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
        sw0 = platform.request("switch", 0).i

        # ------------------------------------------------------------------
        # Démarrage (domaine sync, 100 MHz) : reset PHY 1 ms, MMCM 5 ms,
        # nouvelle tentative toutes les 50 ms tant que le MMCM n'est pas calé.
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
            m.d.sync += boot.eq(0)
        m.d.sync += [phy_rst.eq(boot < T_PHY_RST), mmcm_rst.eq(boot < T_MMCM_RST)]
        m.d.comb += rst_buf.o.eq(~phy_rst)

        # ------------------------------------------------------------------
        # Horloge ULPI : 60 MHz du PHY -> MMCM (compensation, zéro délai) -> BUFG
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

        sw0_u = Signal()
        m.submodules.sw_sync = FFSynchronizer(sw0, sw0_u, o_domain="ulpi")
        fc_val = Mux(sw0_u, FC_PULLUP, FC_NO_PULLUP)
        otg_val = Mux(sw0_u, OTG_DEVICE, OTG_HOST)

        # ------------------------------------------------------------------
        # Bus ULPI : le lien ne pilote DATA que quand DIR = 0 ; NOOP = 0x00.
        # ------------------------------------------------------------------
        dir_ = dir_buf.i
        nxt = nxt_buf.i
        din = dat_buf.i
        data_o = Signal(8)
        stp = Signal()
        m.d.comb += [dat_buf.oe.eq(~dir_), dat_buf.o.eq(data_o), stp_buf.o.eq(stp)]

        # Programme : (écriture?, adresse)
        steps = [(1, REG_SCRATCH), (1, REG_OTG_CTRL), (1, REG_FUNC_CTRL),
                 (0, 0x00), (0, 0x01), (0, 0x02), (0, 0x03),
                 (0, REG_SCRATCH), (0, REG_FUNC_CTRL)]
        step = Signal(range(len(steps)))
        step_wr = Array(C(w, 1) for w, _ in steps)[step]
        step_addr = Array(C(a, 6) for _, a in steps)[step]
        step_wdata = Array([C(SCRATCH_VAL, 8), otg_val, fc_val] + [C(0, 8)] * (len(steps) - 3))[step]
        res = Array(Signal(8, name=f"res{i}") for i in range(len(steps)))
        fc_written = Signal(8)

        timer = Signal(range(ULPI_HZ // 2 + 1), init=ULPI_HZ // 1000)
        timeout = Signal(10)
        fail = Signal()
        report = Signal()
        state = Signal(4)

        with m.FSM(domain="ulpi"):
            with m.State("WAIT"):
                m.d.comb += state.eq(0)
                with m.If(timer != 0):
                    m.d.ulpi += timer.eq(timer - 1)
                with m.Elif(~dir_):
                    m.d.ulpi += [step.eq(0), fail.eq(0), timeout.eq(0)]
                    m.next = "CMD"

            # TX CMD : 0b10aaaaaa RegWrite, 0b11aaaaaa RegRead ; maintenu jusqu'à NXT.
            with m.State("CMD"):
                m.d.comb += [state.eq(1), data_o.eq(Cat(step_addr, step_wr == 0, C(1, 1)))]
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "ABORT"          # le PHY prend le bus (RX CMD ou paquet)
                with m.Elif(nxt):
                    m.d.ulpi += timeout.eq(0)
                    with m.If(step_wr):
                        m.next = "WR_DATA"
                    with m.Else():
                        m.next = "RD_TURN"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            with m.State("WR_DATA"):
                m.d.comb += [state.eq(2), data_o.eq(step_wdata)]
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "ABORT"
                with m.Elif(nxt):
                    with m.If(step_addr == REG_FUNC_CTRL):
                        m.d.ulpi += fc_written.eq(step_wdata)
                    m.next = "WR_STP"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            with m.State("WR_STP"):
                m.d.comb += [state.eq(3), stp.eq(1)]
                m.next = "NEXT"

            with m.State("RD_TURN"):
                m.d.comb += state.eq(4)
                m.d.ulpi += timeout.eq(timeout + 1)
                with m.If(dir_):
                    m.next = "RD_DATA"
                with m.Elif(timeout == 1023):
                    m.d.ulpi += fail.eq(1)
                    m.next = "NEXT"

            with m.State("RD_DATA"):
                m.d.comb += state.eq(5)
                m.d.ulpi += res[step].eq(din)
                with m.If(nxt):
                    m.next = "ABORT"
                with m.Else():
                    m.next = "RD_END"

            with m.State("RD_END"):
                m.d.comb += state.eq(6)
                with m.If(~dir_):
                    m.next = "NEXT"

            with m.State("ABORT"):
                m.d.comb += state.eq(7)
                with m.If(~dir_):
                    m.d.ulpi += timeout.eq(0)
                    m.next = "CMD"

            with m.State("NEXT"):
                m.d.comb += state.eq(8)
                with m.If(step == len(steps) - 1):
                    m.d.ulpi += timer.eq(ULPI_HZ // 2)
                    m.d.comb += report.eq(1)
                    m.next = "WAIT"
                with m.Else():
                    m.d.ulpi += step.eq(step + 1)
                    m.next = "CMD"

        # ------------------------------------------------------------------
        # Moniteur RX : quand le PHY possède le bus (DIR = 1 depuis au moins un
        # cycle, donc hors cycle de retournement) :
        #   NXT = 0 -> RX CMD (sauf pendant notre propre lecture de registre)
        #   NXT = 1 -> octet de paquet USB reçu
        # ------------------------------------------------------------------
        dir_prev = Signal()
        m.d.ulpi += dir_prev.eq(dir_)
        rx_cmd = Signal()
        rx_data = Signal()
        m.d.comb += [rx_cmd.eq(dir_ & dir_prev & ~nxt & (state != 5)),
                     rx_data.eq(dir_ & dir_prev & nxt)]

        last_rxcmd = Signal(8)
        rxcmd_new = Signal()
        rxc_cnt = Signal(16)
        pkt_cnt = Signal(16)
        with m.If(rx_cmd):
            m.d.ulpi += [last_rxcmd.eq(din), rxc_cnt.eq(rxc_cnt + 1)]
            with m.If(din != last_rxcmd):
                m.d.ulpi += rxcmd_new.eq(1)
        with m.If(rx_data):
            m.d.ulpi += pkt_cnt.eq(pkt_cnt + 1)

        # ------------------------------------------------------------------
        # Résultats
        # ------------------------------------------------------------------
        vid = Cat(res[3], res[4])
        pid = Cat(res[5], res[6])
        scr = res[7]
        fc = res[8]
        ok = Signal()
        m.d.comb += ok.eq(~fail & (vid == USB3300_VID) & (pid == USB3300_PID)
                          & (scr == SCRATCH_VAL) & (fc == fc_written))
        regs_ok = Signal()
        with m.If(report):
            m.d.ulpi += regs_ok.eq(ok)
        vbus_ok = Signal()
        m.d.comb += vbus_ok.eq(last_rxcmd[2:4] == 3)

        # ------------------------------------------------------------------
        # UART : message A (périodique) et message B (nouveau RX CMD)
        # ------------------------------------------------------------------
        m.submodules.tx = tx = DomainRenamer("ulpi")(UartTx(round(ULPI_HZ / BAUD)))

        msg_a = (text("VID=") + hexchars(vid, 4) + text(" PID=") + hexchars(pid, 4)
                 + text(" SCR=") + hexchars(scr, 2) + text(" FC=") + hexchars(fc, 2)
                 + text(" RXC=") + hexchars(rxc_cnt, 4) + text(" PKT=") + hexchars(pkt_cnt, 4)
                 + text(" ") + [Mux(ok, ord("O"), ord("E")), Mux(ok, ord("K"), ord("R")),
                                Mux(ok, ord(" "), ord("R"))]
                 + text("\r\n"))
        ls = last_rxcmd[0:2]
        ls_char = Array([C(ord("0"), 8), C(ord("J"), 8), C(ord("K"), 8), C(ord("1"), 8)])[ls]
        msg_b = (text("RXCMD=") + hexchars(last_rxcmd, 2)
                 + text(" LS=") + [ls_char]
                 + text(" VB=") + [last_rxcmd[2:4] + ord("0")]
                 + text(" EV=") + [last_rxcmd[4:6] + ord("0")]
                 + text("\r\n"))

        sel_b = Signal()
        idx = Signal(range(max(len(msg_a), len(msg_b)) + 1))
        ch = Mux(sel_b, Array(msg_b)[idx], Array(msg_a)[idx])
        msg_len = Mux(sel_b, len(msg_b), len(msg_a))
        pend_a = Signal()
        with m.If(report):
            m.d.ulpi += pend_a.eq(1)

        with m.FSM(domain="ulpi"):
            with m.State("IDLE"):
                with m.If(pend_a):
                    m.d.ulpi += [pend_a.eq(0), sel_b.eq(0), idx.eq(0)]
                    m.next = "CHAR"
                with m.Elif(rxcmd_new):
                    m.d.ulpi += [rxcmd_new.eq(0), sel_b.eq(1), idx.eq(0)]
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
                    with m.If(idx == msg_len):
                        m.next = "IDLE"
                    with m.Else():
                        m.next = "CHAR"

        # ------------------------------------------------------------------
        # LED, UART
        # ------------------------------------------------------------------
        beat = Signal(27)
        m.d.sync += beat.eq(beat + 1)
        m.d.comb += [
            leds[0].eq(beat[-1]),
            leds[1].eq(locked_sys),
            leds[2].eq(regs_ok),
            leds[3].eq(vbus_ok),
            uart.tx.o.eq(tx.tx),
        ]

        # ------------------------------------------------------------------
        # ILA : sondes registrées (noms lisibles, retard identique d'un cycle)
        # ------------------------------------------------------------------
        if self.with_ila:
            p_dir = Signal(name="ulpi_dir")
            p_nxt = Signal(name="ulpi_nxt")
            p_stp = Signal(name="ulpi_stp")
            p_oe = Signal(name="ulpi_oe")
            p_dout = Signal(8, name="ulpi_dout")
            p_din = Signal(8, name="ulpi_din")
            p_state = Signal(4, name="ulpi_fsm")
            p_step = Signal(4, name="ulpi_step")
            p_rxcmd = Signal(name="ulpi_rxcmd")
            p_rxdata = Signal(name="ulpi_rxdata")
            m.d.ulpi += [p_dir.eq(dir_), p_nxt.eq(nxt), p_stp.eq(stp), p_oe.eq(dat_buf.oe),
                         p_dout.eq(data_o), p_din.eq(din), p_state.eq(state),
                         p_step.eq(step), p_rxcmd.eq(rx_cmd), p_rxdata.eq(rx_data)]
            # Licence BASIC : 5 sondes au maximum
            m.submodules.ila = Instance("ila_ulpi",
                i_clk=ClockSignal("ulpi"),
                i_probe0=Cat(p_dir, p_nxt, p_stp, p_oe),
                i_probe1=p_dout,
                i_probe2=p_din,
                i_probe3=p_state,
                i_probe4=Cat(p_step, p_rxcmd, p_rxdata),
            )
        return m


ILA_IP_TCL = """
file mkdir ip
create_ip -name ila -vendor xilinx.com -library ip -module_name ila_ulpi -dir [file normalize ip]
set_property -dict [list \\
    CONFIG.C_NUM_OF_PROBES {5} \\
    CONFIG.C_DATA_DEPTH {4096} \\
    CONFIG.C_INPUT_PIPE_STAGES {0} \\
    CONFIG.C_PROBE0_WIDTH {4} \\
    CONFIG.C_PROBE1_WIDTH {8} \\
    CONFIG.C_PROBE2_WIDTH {8} \\
    CONFIG.C_PROBE3_WIDTH {4} \\
    CONFIG.C_PROBE4_WIDTH {6} \\
] [get_ips ila_ulpi]
set_property GENERATE_SYNTH_CHECKPOINT false [get_files ila_ulpi.xci]
generate_target all [get_ips ila_ulpi]
"""

ILA_LTX_TCL = f"write_debug_probes -force {NAME}.ltx"

PROGRAM_TCL = """open_hw_manager
connect_hw_server
open_hw_target
set dev [lindex [get_hw_devices xc7a35t*] 0]
current_hw_device $dev
set_property PROGRAM.FILE {{{bit}}} $dev
{probes}program_hw_devices $dev
refresh_hw_device $dev
close_hw_manager
"""


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--program", action="store_true", help="programmer la carte après la construction")
    ap.add_argument("--no-build", action="store_true", help="générer seulement les fichiers (sans Vivado)")
    ap.add_argument("--no-ila", action="store_true", help="construire sans ILA")
    ap.add_argument("--program-only", action="store_true",
                    help="programmer la carte avec build/ (bit + ltx) sans reconstruire")
    args = ap.parse_args()

    with_ila = not args.no_ila
    overrides = dict(script_after_read=ILA_IP_TCL, script_after_route=ILA_LTX_TCL) if with_ila else {}

    platform = ArtyA7_35Platform()
    platform.add_resources(ULPI_RESOURCE)
    if args.program_only:
        args.program = True
        if not os.path.exists(os.path.join("build", NAME + ".ltx")):
            with_ila = False
    elif args.no_build:
        platform.prepare(Top(with_ila), name=NAME, **overrides).extract("build")
    else:
        platform.build(Top(with_ila), name=NAME, build_dir="build", **overrides)

    if args.program:
        bit = os.path.abspath(os.path.join("build", NAME + ".bit")).replace("\\", "/")
        ltx = bit[:-4] + ".ltx"
        probes = (f"set_property PROBES.FILE {{{ltx}}} $dev\n"
                  f"set_property FULL_PROBES.FILE {{{ltx}}} $dev\n") if with_ila else ""
        tcl = os.path.join("build", "program.tcl")
        with open(tcl, "w") as f:
            f.write(PROGRAM_TCL.format(bit=bit, probes=probes))
        vivado = os.environ.get("VIVADO") or ("vivado.bat" if os.name == "nt" else "vivado")
        subprocess.run([vivado, "-mode", "batch", "-nojournal", "-nolog", "-source", tcl], check=True)