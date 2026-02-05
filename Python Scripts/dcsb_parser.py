#!/usr/bin/env python3
"""Lightweight DCS-BIOS packet framer.

Finds 0x55 0x55 0x55 0x55 headers and emits framed packets (header..before-next-header).
"""
import time

HEADER = b"\x55\x55\x55\x55"


class DcsbiosFramer:
    def __init__(self, on_packet):
        """
        on_packet(packet_bytes: bytes, timestamp: float) -> None
        """
        self.on_packet = on_packet
        self.buf = bytearray()

    def feed(self, data: bytes):
        """Feed raw bytes into the framer. Calls on_packet for each complete frame."""
        if not data:
            return
        self.buf.extend(data)
        while True:
            start = self.buf.find(HEADER)
            if start == -1:
                # keep a small tail in case the header is split across recv calls
                if len(self.buf) > 3:
                    self.buf = self.buf[-3:]
                break
            # if header not at position 0, drop earlier garbage
            if start > 0:
                del self.buf[:start]
                start = 0
            # search for next header (end marker)
            next_h = self.buf.find(HEADER, start + len(HEADER))
            if next_h == -1:
                # incomplete frame, wait for more data
                break
            # extract packet (from this header up to next header)
            packet = bytes(self.buf[start:next_h])
            ts = time.time()
            try:
                self.on_packet(packet, ts)
            except Exception as e:
                # keep running on callback errors
                print("dcsb_parser: packet callback error:", e)
            # remove emitted packet from buffer
            del self.buf[:next_h]
