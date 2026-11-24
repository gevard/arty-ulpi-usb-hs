"""
ULPI étape 3 — périphérique USB 2.0 High-Speed complet avec LUNA, sur Arty A7-35T
+ module USB3343 (portage depuis la version Waveshare USB3300, tag git usb3300).

Le FPGA se présente au PC comme un port série virtuel (CDC-ACM, pilote usbser
intégré à Windows, aucun pilote à installer) :
    SW0 = 1 : connexion (pull-up D+, chirp High-Speed, énumération)
    SW1 = 0 : écho      — tout octet envoyé par le PC lui est renvoyé
    SW1 = 1 : débit     — le FPGA envoie un flux continu (compteur 8 bits) et
                          absorbe tout ce que le PC envoie, pour mesurer le débit
    SW2 = 1 : Full-Speed seulement (diagnostic : à basculer avant SW0)

VID/PID : 0x1209 / 0x0001 (pid.codes, réservé aux essais).

LED :
    LD4 : battement 100 MHz
    LD5 : MMCM verrouillé sur le 60 MHz du PHY
    LD6 : connecté en High-Speed (éteinte en Full-Speed ou déconnecté)
    LD7 : activité USB (paquets émis ou reçus)

Câblage identique aux étapes 1 et 2 (Pmod JB : STP NXT CLK DIR … RESET ;
Pmod JC : DATA0..7). JB10 = RESETB du USB3343, actif BAS.

UART (port COM de l'Arty, 115200 8N1), toutes les 0,5 s, compteurs 16 bits (hex) :
    IN4/OUT4/PING4 : jetons reçus pour l'endpoint 4 ; IN0 : jetons IN pour l'EP0
    NAK/ACK/NYET/STALL/D0/D1 : réponses émises par le FPGA ; HACK : ACK reçus du PC
    SW1, TXV/TXR, RXV/RXR (valid/ready des flux), SPD (0 = HS, 1 = FS)

ILA (horloge USB 60 MHz, 4096 échantillons, 5 sondes, noms lisibles) :
    ulpi_dir, ulpi_nxt, ulpi_stp, ulpi_oe, ulpi_dout[8], ulpi_din[8],
    usb_speed[2] (0 HS, 1 FS, 2 LS), usb_suspended, usb_reset,
    usb_connect, rx_valid, rx_ready, tx_valid, tx_ready, usb_sof

Construction (même environnement que les étapes précédentes, + luna-usb 0.2.3) :
    python -m pip install luna-usb==0.2.3
    python ulpi_luna_acm.py --program
    python ulpi_luna_acm.py --program-only

Timing ULPI (USB3343, oscillateur 60 MHz commun au PHY et au FPGA) :
    contraintes d'E/S dans build/ulpi_luna_acm_ulpi_timing.xdc,
    rapport des chemins d'E/S dans build/ulpi_luna_acm_ulpi_io.rpt
    python ulpi_luna_acm.py --clk-phase 90 --program   -> horloge interne retardée de 90°
    python ulpi_luna_acm.py --board-ns 2 --program     -> marge carte de 2 ns
    La phase utilisée est rappelée sur l'UART (PH=...).
"""

import argparse
import os
import subprocess
from types import SimpleNamespace

from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer, ResetSynchronizer
from amaranth.build import Resource, Subsignal, Pins, Attrs, Clock
from amaranth_boards.arty_a7 import ArtyA7_35Platform

from luna.gateware.usb.usb2 import USBSpeed
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.usb.usb2.request import StallOnlyRequestHandler
from luna.gateware.usb.usb2.endpoints.stream import USBStreamInEndpoint, USBStreamOutEndpoint
from luna.gateware.usb.devices.acm import ACMRequestHandlers
from usb_protocol.types import USBRequestType
from usb_protocol.emitters import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors import cdc


NAME = "ulpi_luna_acm"
SYS_HZ = 100_000_000
ULPI_HZ = 60_000_000

# Timing ULPI du USB3343 en mode « horloge en entrée » (oscillateur 60 MHz commun
# au PHY et au FPGA), fiche USB334x rév. 1.2, table 4.4 :
#   setup STP/data vers le PHY 3,0 ns min, hold 0,0 ns min,
#   délai de sortie DIR/NXT/data du PHY 0,5 ns min, 6,0 ns max.
# BOARD_NS : marge pour les fils Pmod et l'écart d'arrivée de l'horloge au PHY
# et au FPGA (inconnu : 1 ns par défaut, réglable avec --board-ns).
PHY_SETUP_NS, PHY_HOLD_NS = 3.0, 0.0
PHY_TCO_MIN_NS, PHY_TCO_MAX_NS = 0.5, 6.0


def ulpi_timing_xdc(board_ns):
    """Contraintes d'E/S du bus ULPI, par rapport à l'horloge reçue sur JB3."""
    clk = "ulpi_0__clk__io"
    ins = '[get_ports {ulpi_0__dir__io ulpi_0__nxt__io ulpi_0__data__io[*]}]'
    outs = '[get_ports {ulpi_0__stp__io ulpi_0__data__io[*]}]'
    return "\n".join([
        f"set_input_delay  -clock {clk} -max {PHY_TCO_MAX_NS + board_ns:.2f} {ins}",
        f"set_input_delay  -clock {clk} -min {PHY_TCO_MIN_NS - board_ns:.2f} {ins}",
        f"set_output_delay -clock {clk} -max {PHY_SETUP_NS + board_ns:.2f} {outs}",
        f"set_output_delay -clock {clk} -min {-(PHY_HOLD_NS + board_ns):.2f} {outs}",
        # le reset du PHY est asynchrone
        "set_false_path -to [get_ports ulpi_0__rst__io]",
    ])


# Rapport du timing des E/S ULPI après routage (build/<NAME>_ulpi_io.rpt)
ULPI_REPORT_TCL = (f"report_timing -from [all_inputs] -to [all_outputs] -max_paths 20 "
                   f"-file {NAME}_ulpi_io.rpt\n"
                   f"report_timing -from [get_ports ulpi_0__*] -max_paths 20 -append "
                   f"-file {NAME}_ulpi_io.rpt\n"
                   f"report_timing -to [get_ports ulpi_0__*] -max_paths 20 -append "
                   f"-file {NAME}_ulpi_io.rpt\n"
                   f"report_timing -from [get_ports ulpi_0__*] -hold -max_paths 20 -append "
                   f"-file {NAME}_ulpi_io.rpt")


def mmcm_phase(deg):
    """Déphasage de CLKOUT0 arrondi au pas du MMCM (45° / CLKOUT0_DIVIDE = 2,8125°)."""
    step = 45.0 / 16
    return round((deg % 360) / step) * step

VID = 0x1209
PID = 0x0001
MAX_PACKET = 512            # bulk High-Speed
STATUS_EP = 3
DATA_EP = 4

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


def acm_descriptors():
    """Descripteurs CDC-ACM (repris de luna.gateware.usb.devices.acm)."""
    descriptors = DeviceDescriptorCollection()
    with descriptors.DeviceDescriptor() as d:
        d.idVendor = VID
        d.idProduct = PID
        d.iManufacturer = "AGSIM"
        d.iProduct = "ULPI test (Arty + USB3343)"
        d.iSerialNumber = "0001"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        with c.InterfaceAssociationDescriptor() as ia:
            ia.bFirstInterface = 0
            ia.bInterfaceCount = 2
            ia.bFunctionClass = 0x02
            ia.bFunctionSubClass = 0x02
            ia.bFunctionProtocol = 0x01

        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber = 0
            i.bInterfaceClass = 0x02
            i.bInterfaceSubclass = 0x02
            i.bInterfaceProtocol = 0x01
            i.add_subordinate_descriptor(cdc.HeaderDescriptorEmitter())
            union = cdc.UnionFunctionalDescriptorEmitter()
            union.bControlInterface = 0
            union.bSubordinateInterface0 = 1
            i.add_subordinate_descriptor(union)
            call_management = cdc.CallManagementFunctionalDescriptorEmitter()
            call_management.bDataInterface = 1
            i.add_subordinate_descriptor(call_management)
            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | STATUS_EP
                e.bmAttributes = 0x03
                e.wMaxPacketSize = 64
                e.bInterval = 11

        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber = 1
            i.bInterfaceClass = 0x0A
            i.bInterfaceSubclass = 0x00
            i.bInterfaceProtocol = 0x00
            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | DATA_EP
                e.wMaxPacketSize = MAX_PACKET
            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = DATA_EP
                e.wMaxPacketSize = MAX_PACKET
    # Device Qualifier (obligatoire pour un périphérique High-Speed) :
    # bcdUSB 2.00, classe 0, bMaxPacketSize0 64, 1 configuration
    descriptors.add_descriptor(bytes([10, 6, 0x00, 0x02, 0, 0, 0, 64, 1, 0]), descriptor_type=6)
    return descriptors


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
    def __init__(self, with_ila=True, clk_phase=0.0):
        self.with_ila = with_ila
        self.clk_phase = mmcm_phase(clk_phase)

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
        sw1 = platform.request("switch", 1).i
        sw2 = platform.request("switch", 2).i

        # ------------------------------------------------------------------
        # Démarrage (domaine sync) : reset PHY 1 ms, MMCM 5 ms, nouvel essai
        # toutes les 50 ms tant que le MMCM n'est pas verrouillé.
        # Le reset du PHY reste sous notre contrôle (et pas sous celui de LUNA),
        # car le PHY coupe son horloge pendant le reset.
        # USB3343 : broche RESETB active BAS (le USB3300 avait RESET actif haut).
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
        # Domaine « usb » : 60 MHz du PHY -> MMCM (compensation) -> BUFG
        # ------------------------------------------------------------------
        clkfb = Signal()
        clkfb_bufg = Signal()
        clk0 = Signal()
        m.domains.usb = cd_usb = ClockDomain("usb")
        m.submodules.mmcm = Instance("MMCME2_BASE",
            p_CLKIN1_PERIOD=1e9 / ULPI_HZ,
            p_BANDWIDTH="OPTIMIZED",
            p_DIVCLK_DIVIDE=1,
            p_CLKFBOUT_MULT_F=16.0,
            p_CLKOUT0_DIVIDE_F=16.0,
            # retard de l'horloge interne par rapport à JB3 (option --clk-phase)
            p_CLKOUT0_PHASE=self.clk_phase,
            i_CLKIN1=clk_buf.i,
            i_CLKFBIN=clkfb_bufg,
            o_CLKFBOUT=clkfb,
            o_CLKOUT0=clk0,
            o_LOCKED=locked,
            i_RST=mmcm_rst,
            i_PWRDWN=0,
        )
        m.submodules.bufg_fb = Instance("BUFG", i_I=clkfb, o_O=clkfb_bufg)
        m.submodules.bufg_usb = Instance("BUFG", i_I=clk0, o_O=cd_usb.clk)
        m.submodules.usb_rst = ResetSynchronizer(~locked, domain="usb")

        sw0_u = Signal()
        sw1_u = Signal()
        m.submodules.sw0_sync = FFSynchronizer(sw0, sw0_u, o_domain="usb")
        m.submodules.sw1_sync = FFSynchronizer(sw1, sw1_u, o_domain="usb")
        sw2_u = Signal()
        m.submodules.sw2_sync = FFSynchronizer(sw2, sw2_u, o_domain="usb")

        # ------------------------------------------------------------------
        # Bus ULPI au format attendu par LUNA (data.i/.o/.oe, dir.i, nxt.i,
        # stp.o). Pas de « rst » ni de gestion d'horloge par LUNA : on s'en
        # charge ci-dessus.
        # ------------------------------------------------------------------
        bus = SimpleNamespace(
            name="ulpi",
            clk=SimpleNamespace(i=clk_buf.i),
            dir=SimpleNamespace(i=dir_buf.i),
            nxt=SimpleNamespace(i=nxt_buf.i),
            stp=SimpleNamespace(o=stp_buf.o),
            data=SimpleNamespace(i=dat_buf.i, o=dat_buf.o, oe=dat_buf.oe),
        )

        # ------------------------------------------------------------------
        # Périphérique USB LUNA : endpoint 0 standard + CDC-ACM
        # ------------------------------------------------------------------
        m.submodules.usb = usb = USBDevice(bus=bus, handle_clocking=False)
        control_ep = usb.add_standard_control_endpoint(acm_descriptors())
        control_ep.add_request_handler(ACMRequestHandlers())
        control_ep.add_request_handler(StallOnlyRequestHandler(
            lambda setup: (setup.type == USBRequestType.VENDOR) |
                          (setup.type == USBRequestType.RESERVED)))

        status_ep = USBStreamInEndpoint(endpoint_number=STATUS_EP, max_packet_size=64)
        usb.add_endpoint(status_ep)
        rx_ep = USBStreamOutEndpoint(endpoint_number=DATA_EP, max_packet_size=MAX_PACKET)
        usb.add_endpoint(rx_ep)
        tx_ep = USBStreamInEndpoint(endpoint_number=DATA_EP, max_packet_size=MAX_PACKET)
        usb.add_endpoint(tx_ep)

        rx = rx_ep.stream
        tx = tx_ep.stream
        m.d.comb += usb.connect.eq(sw0_u)
        # SW2 = 1 : Full-Speed seulement (pas de chirp High-Speed), pour le diagnostic
        m.d.comb += usb.full_speed_only.eq(sw2_u)

        # SW1 = 0 : écho ; SW1 = 1 : flux continu + absorption
        counter = Signal(8)
        with m.If(~sw1_u):
            # « last » doit suivre : l'endpoint IN n'envoie un paquet incomplet
            # (moins de 512 octets) que lorsque last = 1. Sans lui, un octet
            # isolé resterait en attente jusqu'à remplir un paquet de 512.
            m.d.comb += [
                tx.valid.eq(rx.valid),
                tx.payload.eq(rx.payload),
                tx.first.eq(rx.first),
                tx.last.eq(rx.last),
                rx.ready.eq(tx.ready),
            ]
        with m.Else():
            m.d.comb += [
                tx.valid.eq(1),
                tx.payload.eq(counter),
                rx.ready.eq(1),
            ]
            with m.If(tx.ready):
                m.d.usb += counter.eq(counter + 1)

        # ------------------------------------------------------------------
        # Analyseur de bus ULPI : compte les jetons reçus et les réponses émises
        # (indépendant de LUNA, à partir des signaux bruts du bus).
        # ------------------------------------------------------------------
        u_dir, u_nxt, u_din, u_dout, u_stp = dir_buf.i, nxt_buf.i, dat_buf.i, dat_buf.o, stp_buf.o
        dir_prev = Signal()
        m.d.usb += dir_prev.eq(u_dir)
        rx_byte = Signal()
        m.d.comb += rx_byte.eq(u_dir & dir_prev & u_nxt)
        idx = Signal(2)
        tok_pid = Signal(8)
        tok_b1 = Signal(8)
        with m.If(u_dir & ~dir_prev):
            m.d.usb += idx.eq(0)
        with m.Elif(rx_byte):
            with m.If(idx != 3):
                m.d.usb += idx.eq(idx + 1)
            with m.If(idx == 0):
                m.d.usb += tok_pid.eq(u_din)
            with m.If(idx == 1):
                m.d.usb += tok_b1.eq(u_din)

        tok_done = Signal()
        tok_ep = Signal(4)
        m.d.comb += [tok_done.eq(rx_byte & (idx == 2)),
                     tok_ep.eq(Cat(tok_b1[7], u_din[0:3]))]
        hs_ack = Signal()
        m.d.comb += hs_ack.eq(rx_byte & (idx == 0) & (u_din == 0xD2))

        in_tx = Signal()
        tx_cmd = Signal()
        m.d.comb += tx_cmd.eq(~u_dir & u_nxt & ~in_tx & (u_dout[6:8] == 0b01))
        with m.If(tx_cmd):
            m.d.usb += in_tx.eq(1)
        with m.Elif(u_stp | u_dir):
            m.d.usb += in_tx.eq(0)

        counters = {}
        def count(name, cond):
            c = Signal(16, name=f"cnt_{name}")
            with m.If(cond):
                m.d.usb += c.eq(c + 1)
            counters[name] = c
        count("IN4", tok_done & (tok_pid == 0x69) & (tok_ep == DATA_EP))
        count("OUT4", tok_done & (tok_pid == 0xE1) & (tok_ep == DATA_EP))
        count("PING4", tok_done & (tok_pid == 0xB4) & (tok_ep == DATA_EP))
        count("IN0", tok_done & (tok_pid == 0x69) & (tok_ep == 0))
        count("NAK", tx_cmd & (u_dout[0:4] == 0xA))
        count("ACK", tx_cmd & (u_dout[0:4] == 0x2))
        count("NYET", tx_cmd & (u_dout[0:4] == 0x6))
        count("STALL", tx_cmd & (u_dout[0:4] == 0xE))
        count("D0", tx_cmd & (u_dout[0:4] == 0x3))
        count("D1", tx_cmd & (u_dout[0:4] == 0xB))
        count("HACK", hs_ack)

        # Rapport UART toutes les 0,5 s (115200 8N1, domaine usb)
        m.submodules.tx_uart = tx_uart = DomainRenamer("usb")(UartTx(round(ULPI_HZ / 115200)))
        msg = []
        for name, c in counters.items():
            msg += text(name + "=") + hexchars(c, 4) + text(" ")
        msg += (text("SW1=") + [sw1_u + ord("0")] + text(" SW2=") + [sw2_u + ord("0")]
                + text(" TXV=") + [tx.valid + ord("0")] + text(" TXR=") + [tx.ready + ord("0")]
                + text(" RXV=") + [rx.valid + ord("0")] + text(" RXR=") + [rx.ready + ord("0")]
                + text(" SPD=") + [usb.speed + ord("0")]
                + text(f" PH={self.clk_phase:.0f}")
                + text("\r\n"))
        midx = Signal(range(len(msg) + 1))
        tick = Signal(range(ULPI_HZ // 2))
        go = Signal()
        m.d.usb += tick.eq(Mux(tick == ULPI_HZ // 2 - 1, 0, tick + 1))
        m.d.comb += go.eq(tick == 0)
        with m.FSM(domain="usb"):
            with m.State("IDLE"):
                with m.If(go):
                    m.d.usb += midx.eq(0)
                    m.next = "CHAR"
            with m.State("CHAR"):
                m.d.comb += [tx_uart.data.eq(Array(msg)[midx]), tx_uart.start.eq(1)]
                with m.If(~tx_uart.busy):
                    m.d.usb += midx.eq(midx + 1)
                    m.next = "GAP"
            with m.State("GAP"):
                m.next = "BUSY"
            with m.State("BUSY"):
                with m.If(~tx_uart.busy):
                    with m.If(midx == len(msg)):
                        m.next = "IDLE"
                    with m.Else():
                        m.next = "CHAR"
        m.d.comb += uart.tx.o.eq(tx_uart.tx)

        # ------------------------------------------------------------------
        # LED (activité étirée sur ~70 ms pour être visible)
        # ------------------------------------------------------------------
        activity = Signal(22)
        with m.If(usb.tx_activity_led | usb.rx_activity_led):
            m.d.usb += activity.eq(2 ** 22 - 1)
        with m.Elif(activity != 0):
            m.d.usb += activity.eq(activity - 1)
        is_hs = Signal()
        m.d.comb += is_hs.eq(sw0_u & (usb.speed == USBSpeed.HIGH) & ~usb.suspended)

        beat = Signal(27)
        m.d.sync += beat.eq(beat + 1)
        m.d.comb += [
            leds[0].eq(beat[-1]),
            leds[1].eq(locked_sys),
            leds[2].eq(is_hs),
            leds[3].eq(activity != 0),
        ]

        # ------------------------------------------------------------------
        # ILA (5 sondes au maximum avec la licence BASIC)
        # ------------------------------------------------------------------
        if self.with_ila:
            def reg(width, name, value):
                s = Signal(width, name=name)
                m.d.usb += s.eq(value)
                return s
            p_dir = reg(1, "ulpi_dir", dir_buf.i)
            p_nxt = reg(1, "ulpi_nxt", nxt_buf.i)
            p_stp = reg(1, "ulpi_stp", stp_buf.o)
            p_oe = reg(1, "ulpi_oe", dat_buf.oe)
            p_dout = reg(8, "ulpi_dout", dat_buf.o)
            p_din = reg(8, "ulpi_din", dat_buf.i)
            p_speed = reg(2, "usb_speed", usb.speed)
            p_susp = reg(1, "usb_suspended", usb.suspended)
            p_reset = reg(1, "usb_reset", usb.reset_detected)
            p_conn = reg(1, "usb_connect", usb.connect)
            p_rxv = reg(1, "rx_valid", rx.valid)
            p_rxr = reg(1, "rx_ready", rx.ready)
            p_txv = reg(1, "tx_valid", tx.valid)
            p_txr = reg(1, "tx_ready", tx.ready)
            p_sof = reg(1, "usb_sof", usb.sof_detected)
            m.submodules.ila = Instance("ila_ulpi",
                i_clk=ClockSignal("usb"),
                i_probe0=Cat(p_dir, p_nxt, p_stp, p_oe),
                i_probe1=p_dout,
                i_probe2=p_din,
                i_probe3=Cat(p_speed, p_susp, p_reset),
                i_probe4=Cat(p_conn, p_rxv, p_rxr, p_txv, p_txr, p_sof),
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
    ap.add_argument("--clk-phase", type=float, default=0.0,
                    help="retard de l'horloge interne en degrés (pas de 2,8125°, 360° = 16,7 ns)")
    ap.add_argument("--board-ns", type=float, default=1.0,
                    help="marge carte/fils Pmod des contraintes d'E/S ULPI (ns, défaut 1,0)")
    ap.add_argument("--program-only", action="store_true",
                    help="programmer la carte avec build/ (bit + ltx) sans reconstruire")
    args = ap.parse_args()

    with_ila = not args.no_ila
    overrides = dict(script_after_route=ULPI_REPORT_TCL)
    if with_ila:
        overrides.update(script_after_read=ILA_IP_TCL,
                         script_after_route=ULPI_REPORT_TCL + "\n" + ILA_LTX_TCL)

    platform = ArtyA7_35Platform()
    platform.add_resources(ULPI_RESOURCE)
    # contraintes d'E/S ULPI : fichier XDC séparé (l'Arty utilise déjà add_constraints)
    platform.add_file(NAME + "_ulpi_timing.xdc", ulpi_timing_xdc(args.board_ns))
    # LUNA déduit la présence de VBUS du dernier RX CMD reçu. Sur ce banc, le PHY
    # envoie son RX CMD « VBUS valide » juste après son reset, pendant que le
    # domaine USB attend encore le verrouillage du MMCM : il est perdu, et LUNA
    # croit VBUS absent jusqu'au prochain débranchement/rebranchement du câble.
    # Le module étant alimenté par le bus, on ignore la détection VBUS du PHY.
    platform.ignore_phy_vbus = True
    if args.program_only:
        args.program = True
        if not os.path.exists(os.path.join("build", NAME + ".ltx")):
            with_ila = False
    elif args.no_build:
        platform.prepare(Top(with_ila, args.clk_phase), name=NAME, **overrides).extract("build")
    else:
        platform.build(Top(with_ila, args.clk_phase), name=NAME, build_dir="build", **overrides)

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