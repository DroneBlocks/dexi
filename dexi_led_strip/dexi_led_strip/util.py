from busio import SPI as _SPI
from board import SPI as SPI0

# Configuration for the second SPI buss on the RPI4
SPI1 = lambda: _SPI(21, 20, 19)

# Easy way to get the bus object by port ID
SPI = [SPI0, SPI1]
