#!/usr/bin/env python3
"""Watch the DCS-BIOS messages a panel sends to the sim.

    python tools/panel_monitor.py monitor

Note on DTR: the Pico SDK only reads stdin and writes stdout while the host
asserts DTR (tud_cdc_connected()). pyserial asserts it by default, but a
plain .NET SerialPort does not, which is why a naive capture sees nothing.
"""

import argparse
import sys

import serial


def open_port(port, baud=115200):
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2, write_timeout=1.0)
    # Be explicit: the Pico SDK ignores the CDC entirely until DTR is set.
    ser.dtr = True
    ser.rts = True
    return ser


def cmd_monitor(args):
    ser = open_port(args.port)
    print("listening on %s (Ctrl-C to stop)" % args.port, file=sys.stderr)
    try:
        while True:
            line = ser.readline()
            if line:
                sys.stdout.write(line.decode("utf-8", "replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM11", help="serial port of the panel")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("monitor", help="print the panel's DCS-BIOS messages")

    args = parser.parse_args()
    cmd_monitor(args)


if __name__ == "__main__":
    main()
