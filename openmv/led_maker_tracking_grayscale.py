# Tracking LED marker
#
# Find "maybe" LED marker points with find_blobs and send them with SPI.

import sensor, image, time, struct, pyb
from pyb import SPI

# Some configurations
enable_masking = False
enable_bounded_roi = False

# A class for masking persistent blobs
class Persist:
    def __init__(self):
        self.PL = bytearray([0 for i in range(0, 40*30)])
        return

    def lighten(self, x, y, w, h):
        for i in range((x >> 3), ((x + w + 7) >> 3)):
            for j in range((y >> 3), ((y + h + 7) >> 3)):
                if self.PL[i] < 64:
                    self.PL[i + 40*j] += 1
        return

    def darken_all(self):
        for i in range(0, 40*30):
            if self.PL[i] > 0 and self.PL[i] != 64:
                self.PL[i] -= 1
        return

    def ispersist(self, x, y):
        if self.PL[(x >> 3) + 40*(y >> 3)] > 32:
            return True
        return False

# Grayscale is a bit fast then BGR565.
# Grayscale Tracking Thresholds (Min, Max)
thresholds = [(245, 255)]
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
#thresholds = [(96, 100, -32, 64, -80, 20)] # led threshold
# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene with 16 thresholds before color thresholds start to overlap heavily.

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_auto_gain(False, gain_db=10) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

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
window = (0, 0, 320, 240)
box = [0, 0, 0, 0]
bounded = False
rbuf = bytearray(12)

mask = Persist()

if enable_masking:
    # Make mask for persistent blobs
    for pcount in range(0, 255):
        clock.tick()
        img = sensor.snapshot()
        for blob in img.find_blobs(thresholds, roi=window, pixels_threshold=1, area_threshold=1):
            mask.lighten(blob.x(), blob.y(), blob.w(), blob.h())

while(True):
    clock.tick()
    bcount = 0
    img = sensor.snapshot()
    #mask.darken_all()
    # Looks lens_corr to be heavy. The next commented-out line is for test purpose only.
    # img.lens_corr(strength=1.8, zoom=1.0)
    area = tuple(box) if (enable_bounded_roi and bounded) else window
    for blob in img.find_blobs(thresholds, roi=area, pixels_threshold=1, area_threshold=1, x_stride=1):
        #st mask.lighten(blob.x(), blob.y(), blob.w(), blob.h())
        if (enable_masking and mask.ispersist(blob.cx(), blob.cy())):
            img.draw_rectangle(blob.rect(), color=(0,255,0))
            continue
        else:
            img.draw_cross(blob.cx(), blob.cy())
        str = struct.pack('<HHHHHH', fcount, bcount, blob.cx(), blob.cy(), blob.w(), blob.h())
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
    rcv = struct.unpack('<hhhhhh', rbuf)
    #print(rcv)
    #img.draw_string(160, 120, '% 4d'%(rcv[5]), color=(255,255,255))
    #img.draw_cross(rcv[1], rcv[2], color=(0, 0, 255))
    if (enable_bounded_roi and rcv[0] == 0 and rcv[1] > 0):
        rho = rcv[4]//2
        box[0] = max(0, rcv[1]-rho)
        box[1] = max(0, rcv[2]-rho)
        box[2] = min(rcv[1]+rho, window[2]) - box[0]
        box[3] = min(rcv[2]+rho, window[3]) - box[1]
        # more checks are needed
        bounded = True
        img.draw_rectangle(tuple(box), color=(255,255,0))
    else:
        bounded = False

    fcount = fcount + 1
    if (fcount > 0x8000):
        fcount = 0
    print(clock.fps())
