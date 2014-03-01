#
# UAVCAN DSDL signature computation
#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

#
# CRC-64-WE
# Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
# Initial value: 0xFFFFFFFFFFFFFFFF
# Poly: 0x42F0E1EBA9EA3693
# Reverse: no
# Output xor: 0xFFFFFFFFFFFFFFFF
# Check: 0x62EC59E3F1A4F00A
#
class Signature:
    MASK64 = 0xFFFFFFFFFFFFFFFF
    POLY = 0x42F0E1EBA9EA3693

    def __init__(self, extend_from=None):
        if extend_from is not None:
            self._crc = (int(extend_from) & Signature.MASK64) ^ Signature.MASK64
        else:
            self._crc = Signature.MASK64

    def add(self, data_bytes):
        if isinstance(data_bytes, str):
            data_bytes = map(ord, data_bytes)
        for b in data_bytes:
            self._crc ^= (b << 56) & Signature.MASK64
            for _ in range(8):
                if self._crc & (1 << 63):
                    self._crc = ((self._crc << 1) & Signature.MASK64) ^ Signature.POLY
                else:
                    self._crc <<= 1

    def get_value(self):
        return (self._crc & Signature.MASK64) ^ Signature.MASK64


def compute_signature(data):
    s = Signature()
    s.add(data)
    return s.get_value()


# if __name__ == '__main__':
if 1:
    s = Signature()
    s.add(b'123')
    s.add('456789')
    assert s.get_value() == 0x62EC59E3F1A4F00A
