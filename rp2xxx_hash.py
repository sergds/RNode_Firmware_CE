# This utility is used to calculate hashes for RP2XXX-based devices,
# it takes account of mutable BTstack TLV storage (which is part of .rodata, aligned to flash sector size),
# and implements almost the same algorithm as in firmware.
# (C) 2026 Sergey Morozyuk <me@sergds.xyz>

import sys
import os
import hashlib
from elftools.elf.elffile import ELFFile 

CHUNK_SIZE = 512
FLASH_BASE = 0x10000000
BIN_BASENAME = "RNode_Firmware_CE.ino"

tlv_base: int = 0x0
hasBtstack = False

if len(sys.argv) == 1:
    print(f"usage: {sys.argv[0]} path/to/builddir")
    exit(1)

elf_path = os.path.join(sys.argv[1], f"{BIN_BASENAME}.elf")
bin_path = os.path.join(sys.argv[1], f"{BIN_BASENAME}.bin")

with open(elf_path, "rb") as elffile:
    elf = ELFFile(elffile)
    symtab = elf.get_section_by_name(".symtab")
    for sym in symtab.iter_symbols():
        if "cyw43" in sym.name and not hasBtstack:
            print("This binary has CYW43 driver, enabling TLV bypass")
            hasBtstack = True
        if "__bluetooth_tlv" in sym.name:
            tlv_base = sym.entry["st_value"]
            print(f"Found BTstack TLV: base={hex(tlv_base)}, symbol={sym.name}, .bin offset={hex(tlv_base - 0x10000000)}")
            tlv_base = tlv_base - FLASH_BASE

if tlv_base == 0 and hasBtstack:
    print("Failed to find BTstack TLV storage base address! Did something change significantly?")
    exit(1)

with open(bin_path, "rb") as f:
    hash = hashlib.new("SHA256")
    skip: int = 0

    while True:
        chunk = f.read(512)
        if skip > 0 and hasBtstack:
            skip -= 1
            continue
        if len(chunk) == 0:
            break
        if f.tell() == tlv_base and hasBtstack:
            skip = int((2 * 4096) / CHUNK_SIZE)
            print(f"skipping {skip} chunks from {hex(f.tell())} to {hex(f.tell() + CHUNK_SIZE*skip)}")
        hash.update(chunk)
    
    print(hash.hexdigest())