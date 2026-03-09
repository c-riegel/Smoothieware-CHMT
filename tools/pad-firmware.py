#!/usr/bin/env python3
"""Pad a firmware binary to 512KB (0x80000 bytes) with 0xFF.

st-flash v1.8.0 has a bug when used with ST-Link V3 programmers: the flash
loader crashes when writing binaries smaller than the full 512KB flash size.
This script pads the binary so st-flash can write it successfully.

Usage:
    python3 tools/pad-firmware.py STM32F407xG/main.bin

Produces STM32F407xG/main-padded.bin (524288 bytes).
"""
import sys

FLASH_SIZE = 524288  # 512KB

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <firmware.bin>")
    sys.exit(1)

infile = sys.argv[1]

with open(infile, 'rb') as f:
    data = f.read()

if len(data) > FLASH_SIZE:
    print(f"Error: {infile} is {len(data)} bytes, exceeds {FLASH_SIZE} byte flash size")
    sys.exit(1)

if len(data) == FLASH_SIZE:
    print(f"{infile}: already {FLASH_SIZE} bytes, no padding needed")
    sys.exit(0)

padded = data + b'\xff' * (FLASH_SIZE - len(data))
outfile = infile.replace('.bin', '-padded.bin')

with open(outfile, 'wb') as f:
    f.write(padded)

print(f"{infile}: {len(data)} bytes -> {outfile}: {len(padded)} bytes")
