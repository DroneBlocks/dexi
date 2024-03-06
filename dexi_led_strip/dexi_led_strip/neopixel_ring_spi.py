from typing import Any, Sequence

from busio import SPI
from neopixel_spi import NeoPixel_SPI
from adafruit_pixelbuf import PixelBuf, ColorUnion


class PixelbufView:
    def __init__(self, pixel_buf: PixelBuf, force_no_auto_write: bool = False) -> None:
        self.parent = pixel_buf
        self.force_no_auto_write = force_no_auto_write

    @staticmethod
    def parse_byteorder(byteorder: str) -> tuple[int, str, bool, bool]:
        return PixelBuf.parse_byteorder(byteorder)

    @property
    def bpp(self) -> None:
        return self.parent.bpp

    @property
    def brightness(self) -> None:
        return self.parent.brightness

    @brightness.setter
    def brightness(self, value: float) -> None:
        self.parent.brightness = value

    @property
    def byteorder(self) -> None:
        return self.parent.byteorder

    def __len__(self) -> int:
        return len(self.parent)

    def show(self) -> Any:
        return self.parent.show()

    def fill(self, color: ColorUnion) -> None:
        self.parent.fill(color)

    @property
    def auto_write(self) -> bool:
        if self.force_no_auto_write:
            return False
        else:
            return self.parent.auto_write

    @auto_write.setter
    def auto_write(self, state: bool) -> None:
        if self.force_no_auto_write:
            state = False
        self.parent.auto_write = state

    def __setitem__(
        self, index: int | slice, val: ColorUnion | Sequence[ColorUnion]
    ) -> None:
        if isinstance(index, slice):
            start, stop, step = index.indices(self.parent._pixels)
            for val_i, in_i in enumerate(range(start, stop, step)):
                r, g, b, w = self.parent._parse_color(val[val_i])
                self.parent._set_item(in_i, r, g, b, w)
        else:
            r, g, b, w = self.parent._parse_color(val)
            self.parent._set_item(index, r, g, b, w)
    
        if self.auto_write:
            self.show()

    def __getitem__(self, index: int | slice) -> list[ColorUnion]:
        if isinstance(index, slice):
            out = []
            for in_i in range(*index.indices(len(self.parent._post_brightness_buffer) // self.bpp)):
                out.append(self.parent._getitem(in_i))
            return out
        if index < 0:
            index += len(self)
        if index >= self._pixels or index < 0:
            raise IndexError
        return self.parent._getitem(index)
    
    
class ReversedPixelbufView(PixelbufView):
    def _map_index(self, value: int):
        value = -value - 1
        value %= len(self)

        return value

    def __setitem__(
        self, index: int | slice, val: ColorUnion | Sequence[ColorUnion]
    ) -> None:
        if isinstance(index, slice):
            start, stop, step = index.indices(self.parent._pixels)
            for val_i, in_i in enumerate(range(start, stop, step)):
                r, g, b, w = self.parent._parse_color(val[val_i])
                self.parent._set_item(self._map_index(in_i), r, g, b, w)
        else:
            r, g, b, w = self.parent._parse_color(val)
            self.parent._set_item(self._map_index(index), r, g, b, w)
    
        if self.auto_write:
            self.show()

    def __getitem__(self, index: int | slice) -> list[ColorUnion]:
        if isinstance(index, slice):
            out = []
            for in_i in range(*index.indices(len(self.parent._post_brightness_buffer) // self.bpp)):
                out.append(self.parent._getitem(self._map_index(in_i)))
            return out
        if index < 0:
            index += len(self)
        if index >= self._pixels or index < 0:
            raise IndexError
        return self.parent._getitem(self._map_index(index))


class MirroredPixelbufView(PixelbufView):
    def __init__(self, pixel_buf: PixelBuf) -> None:
        super().__init__(pixel_buf)

        self.normal = PixelbufView(self.parent, True)
        self.reversed = ReversedPixelbufView(self.parent, True)

        self._length = len(self.parent) // 2
        if len(self.parent) & 0x1 != 0:
            self._length += 1

    def __len__(self) -> int:
        return self._length

    def __setitem__(
        self, index: int | slice, val: ColorUnion | Sequence[ColorUnion]
    ) -> None:
        if isinstance(index, slice):
            start, stop, _ = index.indices(len(self))
            assert 0 <= start
            assert stop < len(self)
        else:
            assert 0 <= index < len(self)

        self.normal[index] = val
        self.reversed[index] = val
    
        if self.auto_write:
            self.show()


class NeoPixelRing_SPI(NeoPixel_SPI):
    def __init__(
        self,
        spi: SPI,
        n: int,
        bpp: int = 3,
        brightness: float = 1.0,
        auto_write: bool = True,
        pixel_order: str | tuple[int, ...] | None = None,
        frequency: int = 6400000,
        reset_time: float = 80e-6,
        bit0: int = 0b10000000, # 0b11000000,
        bit1: int = 0b11110000,
        start_index: int = 0
    ) -> None:
        super().__init__(
            spi,
            n,
            bpp=bpp,
            brightness=brightness,
            auto_write=auto_write,
            pixel_order=pixel_order,
            frequency=frequency,
            reset_time=reset_time,
            bit0=bit0,
            bit1=bit1
        )
        self._start_index = start_index

    def _map_index(self, value: int) -> int:
        assert 0 <= value < self._pixels
    
        value += self._start_index
        value %= self._pixels
    
        return value

    def _set_item(self, index: int, r: int, g: int, b: int, w: int) -> None:
        super()._set_item(self._map_index(index), r, g, b, w)
