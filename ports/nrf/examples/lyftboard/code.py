import time
import board
from digitalio import DigitalInOut, Direction, Pull
import neopixel
import busio
from adafruit_epd.epd import Adafruit_EPD
from adafruit_epd.il0373 import Adafruit_IL0373
import adafruit_lsm9ds1

# Configure the setup
PIXEL_PIN = board.P0_15   # pin that the NeoPixel is connected to
ORDER = neopixel.RGB   # pixel color channel order
COLOR = (100, 50, 150) # color to blink
CLEAR = (0, 0, 0)      # clear (or second color)
DELAY = 0.25           # blink rate in seconds

led_brake = DigitalInOut(board.P1_06)
led_hl1 = DigitalInOut(board.P1_07)
led_hl2 = DigitalInOut(board.P1_05)

en_3v3a = DigitalInOut(board.P1_03)
en_3v3a.pull = Pull.DOWN
#en_3v3a.direction = Direction.OUTPUT
#en_3v3a.value = False

led_brake.direction = Direction.OUTPUT
led_hl1.direction = Direction.OUTPUT
led_hl2.direction = Direction.OUTPUT

print("Setup complete")

# Create the NeoPixel object
pixel = neopixel.NeoPixel(PIXEL_PIN, 2, pixel_order=ORDER, brightness=0.2)
pixel[0] = (0, 40, 10)
pixel[1] = (0, 30, 50)

i2c = busio.I2C(board.SCL, board.SDA)
while not i2c.try_lock():
    pass

print("I2C addresses found:", [hex(device_address)
                                   for device_address in i2c.scan()])

#sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
ecs = DigitalInOut(board.P0_30)
dc = DigitalInOut(board.P0_28)
rst = DigitalInOut(board.P0_02)
busy = DigitalInOut(board.P0_03)

display = Adafruit_IL0373(152, 152, spi, cs_pin=ecs, dc_pin=dc, rst_pin=rst,
                          busy_pin=busy, sramcs_pin=None)

display.fill(Adafruit_EPD.WHITE)

#display.fill_rect(20, 20, 50, 60, Adafruit_EPD.RED)
#display.hline(80, 30, 60, Adafruit_EPD.BLACK)
#display.vline(80, 30, 60, Adafruit_EPD.BLACK)

#display.display()

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    pos -= 170
    return (pos * 3, 0, 255 - pos * 3)

ind = 0
while True:
    led_brake.value = False
    led_hl1.value = False
    led_hl2.value = True

    time.sleep(0.1)

    led_brake.value = True
    led_hl1.value = True
    led_hl2.value = False
    pixel[0] = wheel(ind)
    pixel[1] = wheel(255-ind)
    ind += 1
    if ind >= 255:
        ind = 0
    #accel_x, accel_y, accel_z = sensor.acceleration
    #mag_x, mag_y, mag_z = sensor.magnetic
    #gyro_x, gyro_y, gyro_z = sensor.gyro
    #temp = sensor.temperature
    # Print values.
    #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    #print('Temperature: {0:0.3f}C'.format(temp))

    time.sleep(0.1)
