"""Original small DDS headers/pixels for parser and local-server tests."""
import struct


def dds(width=4, height=4, fourcc=b'DXT1', payload=None, bits=0, masks=(0, 0, 0, 0),
        pf_flags=None, flags=0x1007, pitch=0, mips=1, caps2=0):
    header = [0] * 31
    header[0:7] = [124, flags, height, width, pitch, 0, mips]
    header[18:26] = [32, pf_flags if pf_flags is not None else 4, int.from_bytes(fourcc, 'little'), bits, *masks]
    header[26:28] = [0x1000, caps2]
    if payload is None:
        payload = struct.pack('<HHI', 0xf800, 0x07e0, 0)
    return b'DDS ' + struct.pack('<31I', *header) + payload
