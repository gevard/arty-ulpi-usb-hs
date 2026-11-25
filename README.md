# ULPI High-Speed USB device on Arty A7 (USB3343 / USB3300)

A USB 2.0 **High-Speed** device built on a Digilent **Arty A7-35T** FPGA board and an
external **ULPI PHY** wired to Pmod connectors **JB** and **JC**. The gateware is written in
[Amaranth](https://github.com/amaranth-lang/amaranth) and uses the
[LUNA](https://github.com/greatscottgadgets/luna) USB stack. The finished device enumerates
on Windows as a virtual serial port (CDC-ACM, built-in `usbser` driver, no driver to install).

The project goes in three steps, each a separate bitstream:

| Step | Script | What it proves |
|---|---|---|
| 1 | `ulpi_regtest.py` | ULPI register access: reads Vendor/Product ID, writes and reads back the Scratch register |
| 2 | `ulpi_rxcmd.py` | RX CMDs (LineState, VBUS), D+ pull-up control, Full-Speed connection seen by the PC |
| 3 | `ulpi_luna_acm.py` | Complete High-Speed CDC-ACM device (echo and throughput modes) |

Measured on the USB3343 prototype (Windows, High-Speed):

| Test | Result |
|---|---|
| Read (FPGA → PC) | 30.6 MB/s, no gap in the counter stream |
| Write (PC → FPGA) | 43.7 MB/s |
| Echo | 200 transfers of 1 to 4096 bytes, 10.8 MB/s round trip |

> Code comments and UART messages are in French.

## Two PHY versions

| | **USB3343** (own prototype board) | **USB3300** (Waveshare USB3300 module) |
|---|---|---|
| Git ref | `main` (tag `usb3343-hs-ok`) | tag / branch `usb3300` |
| Product ID read in step 1 | `0x0009` | `0x0004` |
| Reset pin | `RESETB`, **active low** | `RESET`, active high |
| ULPI clock mode | **Clock-in**: one 60 MHz oscillator feeds the PHY `REFCLK` *and* FPGA JB3. The PHY `CLKOUT` pin is tied to `VDDIO` to select this mode. | **Clock-out**: the module's crystal drives the PHY PLL, and the PHY outputs 60 MHz on `CLKOUT` → JB3 |
| Extra in this version | ULPI I/O timing constraints, `--clk-phase`, `--board-ns`, UART activity counters, SW2 (Full-Speed only), ILA capture script | — |

To build the USB3300 version: `git checkout usb3300`.

The KiCad files of the USB3343 board will be published once the next revision is tested.

## Wiring (Pmod JB and JC)

Pmod pins 1–4 are on the top row, 7–10 on the bottom row. Pins 5/11 are GND and 6/12 are 3.3 V.
All signals are LVCMOS33.

| Pmod pin | FPGA pin | ULPI signal | Direction (FPGA) | USB3343 pin |
|---|---|---|---|---|
| JB1 | E15 | STP | out | 24 |
| JB2 | E16 | NXT | in | 3 |
| JB3 | D15 | CLK (60 MHz) | in | — (oscillator, also on `REFCLK` pin 21) |
| JB4 | C15 | DIR | in | 1 |
| JB10 | J15 | RESET / RESETB | out | 22 (`RESETB`, active low) |
| JC1 | U12 | DATA0 | in/out | 4 |
| JC2 | V12 | DATA1 | in/out | 5 |
| JC3 | V10 | DATA2 | in/out | 6 |
| JC4 | V11 | DATA3 | in/out | 7 |
| JC7 | U14 | DATA4 | in/out | 8 |
| JC8 | V14 | DATA5 | in/out | 10 |
| JC9 | T13 | DATA6 | in/out | 11 |
| JC10 | U13 | DATA7 | in/out | 12 |
| JB/JC 5, 11 | — | GND | — | 25 (pad) |
| JB/JC 6, 12 | — | 3.3 V | — | 9 (`VDDIO`) |

Notes:

- JB3 (D15) is a clock-capable (MRCC) pin. The 60 MHz clock goes through an MMCM before
  driving the FPGA logic.
- On the USB3343 board, `VBAT` comes from USB VBUS through an inductor. The PHY is therefore
  only powered while the USB cable is plugged in.
- USB3343 support components: `RBIAS` 8.06 kΩ to GND, `RVBUS` 20 kΩ, `ID` tied to `VDD33`
  (device), `XO` left open, `CLKOUT` tied to `VDDIO` (clock-in mode).
- **D+ / D- go straight to the connector, without series resistors.** The PHY already
  contains the 1.5 kΩ pull-up, the 15 kΩ pull-downs and the 45 Ω HS terminations. Check the
  pin order: connector D+ → `DP` (pin 13), connector D- → `DM` (pin 14). See
  [Lessons learned](#lessons-learned).

## Board controls

### Step 3: `ulpi_luna_acm.py` (CDC-ACM device)

| Switch | Function |
|---|---|
| **SW0** | `1` = connect (D+ pull-up, High-Speed chirp, enumeration). `0` = disconnected. A **0 → 1 edge restarts everything** (PHY reset, MMCM, LUNA). |
| **SW1** | `0` = **echo**: every byte sent by the PC is sent back. `1` = **throughput**: the FPGA sends a continuous 8-bit counter and discards everything the PC sends. Can be changed at any time. |
| **SW2** | `1` = Full-Speed only (no HS chirp), for diagnosis. Set it **before** SW0 0 → 1. |
| SW3 | Not used |

| LED | Meaning |
|---|---|
| **LD4** | Heartbeat from the 100 MHz system clock (about 1 Hz): the FPGA is configured |
| **LD5** | MMCM locked on the PHY's 60 MHz clock: the ULPI clock is present |
| **LD6** | Connected in **High-Speed** (off in Full-Speed, while disconnected, or in suspend) |
| **LD7** | USB activity (packets sent or received), stretched to about 70 ms to be visible |

Typical patterns:

- **SW0 = 0**: LD4 blinks and LD5 is on. LD7 may flicker because the PHY reports line noise.
- **SW0 = 1, enumeration OK**: LD6 is on and LD7 is on or flickering. A new COM port appears.
- **LD6 / LD7 blink a few times, then stay off**: the PC retries enumeration and gives up
  (Windows reports code 43). Look at the UART counters.

### Step 2: `ulpi_rxcmd.py`

| Control | Meaning |
|---|---|
| SW0 | `0` = D+/D- pull-downs on, no pull-up. `1` = pull-downs off, 1.5 kΩ pull-up on D+ (Full-Speed device). The PC then sees a device that never answers, which is expected at this step. |
| LD4 / LD5 | Heartbeat / MMCM locked |
| LD6 | Register test OK (IDs, Scratch, Function Control read-back) |
| LD7 | VBUS valid (from the last RX CMD) |

### Step 1: `ulpi_regtest.py`

No switches are used.

| LED | Meaning |
|---|---|
| LD4 / LD5 | Heartbeat / MMCM locked |
| LD6 | Vendor ID = `0x0424` and Product ID as expected |
| LD7 | Scratch register write and read-back OK |

## UART report

The Arty USB-UART port (115200 8N1) prints one line every 0.5 s. Step 3 prints 16-bit
hexadecimal counters:

```
IN4=0000 OUT4=0000 PING4=0000 IN0=0016 NAK=0000 ACK=0023 NYET=0000 STALL=0003 D0=0002 D1=0011
HACK=0013 RXA=3E7F RXE=0000 LS=540F SOF=2960 RST=0001 SW1=0 SW2=0 TXV=0 TXR=1 RXV=0 RXR=1 SPD=0 PH=0
```

| Field | Meaning |
|---|---|
| `IN4` / `OUT4` / `PING4` | Tokens received for the data endpoint (EP4) |
| `IN0` | IN tokens for the control endpoint |
| `NAK` `ACK` `NYET` `STALL` `D0` `D1` | Handshakes and data packets sent by the FPGA |
| `HACK` | ACKs received from the PC |
| `RXA` / `RXE` | Receive starts / `RxError` reported by the PHY |
| `LS` | LineState changes |
| `SOF` | SOF packets seen by LUNA (≈ 8000/s in High-Speed, 1000/s in Full-Speed) |
| `RST` | USB bus resets detected |
| `TXV` `TXR` `RXV` `RXR` | valid/ready of the IN and OUT streams |
| `SPD` | `0` = High-Speed, `1` = Full-Speed |
| `PH` | MMCM clock phase (`--clk-phase`) |

`bus_monitor.py COMx` turns these lines into rates per second, plus alerts in plain text.

A healthy High-Speed device shows `SPD=0`, `SOF` growing by about 4000 per line, `RXE=0`
and `RST=1`.

## Build and program

Requirements (tested versions):

- Python 3.14 with the packages in `requirements.txt`
  (`python -m pip install -r requirements.txt`)
- AMD/Xilinx **Vivado** 2026.1 (free edition), `vivado.bat` in the `PATH` or the `VIVADO`
  environment variable pointing to it

```
python ulpi_luna_acm.py --program        # build, then program the board
python ulpi_luna_acm.py --program-only   # program the last build again
python ulpi_luna_acm.py --no-build       # generate the sources only (no Vivado)
python ulpi_luna_acm.py --no-ila         # build without the ILA
python ulpi_luna_acm.py --clk-phase 90   # delay the internal ULPI clock by 90°
```

Steps 1 and 2 take the same options (`ulpi_regtest.py`, `ulpi_rxcmd.py`).
The bitstream is written to `build/`.

## PC-side tests

```
python test_acm.py echo     # SW0 = 1, SW1 = 0
python test_acm.py read     # SW0 = 1, SW1 = 1 : FPGA -> PC throughput
python test_acm.py write    # SW0 = 1, SW1 = 1 : PC -> FPGA throughput
```

The COM port is found automatically from the USB VID/PID `1209:0001`. This is the
[pid.codes](https://pid.codes) test PID, for private testing only. Use `--port COMx` to
choose the port by hand.

## Debug tools

- **ILA**: every bitstream contains a Vivado ILA on the ULPI bus (60 MHz, 4096 samples,
  5 probes so that it fits the free BASIC license).
- `ila_capture.tcl` (Vivado Tcl console): `capture setup | rxdata | rxerror | reset` arms the
  ILA on a USB event and writes a CSV after SW0 0 → 1. The output paths are written for
  `C:/ulpi`; edit them for your setup.
- `ulpi_decode.py capture.csv`: decodes an ILA CSV export into ULPI commands, RX CMDs and
  packets.

## Lessons learned

- **D+ / D- swapped.** If the PHY's D+ pull-up ends up on the cable's D- line, the PC sees a
  *Low-Speed* device. Symptoms: `LS` changes a lot, `RST` keeps climbing (about 25/s), but
  `RXA = SOF = 0` in both Full- and High-Speed, and Windows reports code 43. Revision 2 of the
  USB3343 board had this error.
- **Zero-length packets.** LUNA's OUT stream endpoint flags `last` only at the end of a
  *short* packet. An echo of a transfer that is an exact multiple of 512 bytes was therefore
  sent back without a ZLP, and Windows `usbser` never delivered the data. The echo now holds
  the last byte and sends it with `last = 1` after 200 µs of silence, so LUNA appends the ZLP.
- **Loose Pmod contact on the clock line.** A bad contact on JB3 gave random behaviour that
  looked like a timing problem. Check every Pmod pin before suspecting the logic.
- **USB334x errata (DS80000645A, module 2).** In High-Speed, the link must wait one clock
  cycle between a ULPI register write and a transmit command, or the chirp fails. LUNA 0.2.3
  works with the USB3343 as is.

## License

BSD 3-Clause, see [LICENSE](LICENSE).
