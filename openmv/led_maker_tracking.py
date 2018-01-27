# Tracking LED marker
#
# Find "maybe" LED marker points with find_blobs and send them with SPI.

import sensor, image, time, struct, pyb
from pyb import SPI

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
thresholds = [(90, 100, -32, 64, -80, 20)] # blue led_thresholds
# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene with 16 thresholds before color thresholds start to overlap heavily.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False, value=30) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

cs = pyb.Pin('P3', pyb.Pin.OUT)
cs.high()
spi = SPI(2, SPI.MASTER, baudrate=16000000, polarity=0, phase=0)
#cs.low()
hs = pyb.Pin('P4', pyb.Pin.IN)

clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.

fcount = 0
window = (0, 0, 230, 240)
rbuf = bytearray(12)

while(True):
    clock.tick()
    bcount = 0
    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, roi=window, pixels_threshold=2, area_threshold=2):
        img.draw_cross(blob.cx(), blob.cy())
        str = struct.pack('<HHHHHH', fcount, bcount, blob.cx(), blob.cy(), blob.pixels(), blob.code())
        while hs.value() == 0:
            pyb.delay(1)
        cs.low()
        pyb.udelay(1)
        spi.send_recv(str, rbuf)
        pyb.udelay(1)
        cs.high()
        pyb.udelay(10)
        bcount = bcount + 1

    # end of frame packet
    while hs.value() == 0:
        pyb.delay(1)
    str = struct.pack('<HHHHHH', 0xa5a5, 0, 0, 0, 0, 0)
    cs.low()
    pyb.udelay(1)
    spi.send_recv(str, rbuf)
    pyb.udelay(1)
    cs.high()
    pyb.udelay(10)

    fcount = fcount + 1
    if (fcount > 0x8000):
        fcount = 0
    print(clock.fps())
