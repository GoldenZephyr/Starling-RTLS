#!/usr/bin/python3

import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 3000000
spi.mode=(0b00)
out = spi.xfer2([0x00, 0x00, 0x00, 0x00, 0x00])
print("%x" % out[1])
print("%x" % out[2])
print("%x" % out[3])
print("%x" % out[4])
