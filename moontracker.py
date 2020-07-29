from collections import namedtuple
from math import pi, sin, cos, tan, asin, atan2, acos
import mooncalc
from network import WLAN
import time

# Find out whether we are running micropython or not also, micropython
# can run on UNIX, too.  so, if UNIX=1 and MICROPYTHON = 0 we're
# running a standard python e.g. on a macbook.

# We need to know because if UNIX=1, the epoch for time.time starts in
# 1970.  otherwise, time epoch starts in 2000.

UNIX = 1
MICROPYTHON = 0

TIME_EPOCH_DIFF = 0
try:
    import sys
    if sys.implementation[0] == 'micropython':
        MICROPYTHON = 1
        # note: should add other platforms here as well.  The unix
        # micropython code uses its host time, so its epoch starts at
        # 1970
        if sys.platform == 'esp32':
            UNIX = 0
            TIME_EPOCH_DIFF = 946684815
except:
    pass

# The formulas we use are slightly off, we loose a few seconds per decade.
# in 2020, DELTA_T is about 72.
# For more, see https://eclipse.gsfc.nasa.gov/SEhelp/deltaT.html
DELTA_T = 71.6

# set if we want to debug
DEBUG = False
FAST_FORWARD_FACTOR = 1 # to simlate faster speed, set to value >1, e.g. 100

# to allow to run this on a local computer for testing,
# only deal with hardware on a MCU board:
if MICROPYTHON == 1 and UNIX == 0:
    from machine import UART, Pin, PWM, RTC
    from neopixel import NeoPixel
    import ntptime
    import tm1638
    import uasyncio as asyncio
    import utime


    tm = tm1638.TM1638(stb=Pin(4), clk=Pin(16), dio=Pin(17))

    # every 2nd LED on
    tm.leds(0b01010101)

    # dim both LEDs and segments
    tm.brightness(0)
    
    # Two servos - one for azimuth, one for altitude.
    # Since the servos only have 180 degree range, I need to do
    # funky things with AZ/ALT
    servo_az = PWM(Pin(5), freq=50, duty=77)
    servo_alt = PWM(Pin(22), freq=50, duty=77)


    from micropyGPS import MicropyGPS

    # Create a GPS module instance.
    uart = UART(1, baudrate=9600)
    uart.init(rx=21, tx=18)

    # Create a GPS module instance.
    mygps = MicropyGPS(location_formatting='dd')

    if 0: # TODO
        # (optional) neopixel led to mounted on pointing arm to indicate if the
        #  moon is up in the sky or not. If not, we turn Neopixel off (this way, we
        # CAN address the lower hemisphere after all)
        npPin = Pin(23, Pin.OUT)
        np = NeoPixel(npPin,1)
else:
    import asyncio

# dummy web server handler, I can add remote functionality to it later
# for now we just return some text for every request.
async def serve(reader, writer):
    # Consume GET line
    reader.readline()

    await writer.awrite(b"HTTP/1.1 200 OK\r\n\r\nTra-la-laaaa!\r\n")
    await writer.aclose()
    

# return PWM value to set Azimuth servo to specified angle.
# Empirically, roughly calibrated my specific servos
# Tower PRO SG90
def angle_to_duty(angle, a, b):
    if angle < 0 or angle > 180:
        print("illegal angle: ", angle)
        return 0
    return a + int((180-angle)/180 * b)


def az_angle_to_duty(angle):
    return angle_to_duty(angle, 32, 143-30)


# Funny how this is the same model servo but needs quite different calibration.
def alt_angle_to_duty(angle):
    return angle_to_duty(angle, 32, 121-32)


# helper functions to print date and time in 4 chars each (for
# blinkenlights)
def mydate():
  lt = utime.localtime()
  return int("{:02d}{:02d}".format(lt[1],lt[2]))


def mytime():
  lt = utime.localtime()
  return int("{:02d}{:02d}".format(lt[3],lt[4]))


# This is the main loop. Every second,
# - compute moon position
# - set servos
# - optionally adjust LED on pointer if we have one
# - update display
# - in debug mode, also send coordinates to console
# - in debug mode, can also fast forward, make moon start at current
#   time but then move it forward X seconds every second.
latitude = 0 # these will be changed by GPS periodically
longitude = 0

async def tracker(delay_ms):
    intensity = 0
    t = time.time()
    while True:
        pos = mooncalc.get_moon_position(t + DELTA_T + TIME_EPOCH_DIFF,
                                         latitude, longitude) # SafeWay
        if DEBUG:
            print("t: {:35} Pos: ({:=3.2f}, {:=3.2f}), Az: {:=7.2f} Alt: {:=7.2f} Dist: {:=9.1f} km".format(
                repr(time.localtime(int(t))),
                latitude, longitude,
                (pos.azimuth * 180 /pi + 180) % 360,
                pos.altitude * 180 /pi, pos.distance))
        
        az = int(round(pos.azimuth * 180 /pi + 180))
        alt = int(round(pos.altitude * 180 /pi))
        # since our servos can only rotate 180 degrees, we need to fiddle
        # with the angles to get correct servo positions:

        # we divide the Sphere in 4 quarter spheres. First, we cut the
        # sphere in upper and lower hemishphere. THe moon is below the
        # horizon in the lower hemissphere, so let's only worry about
        # the upper hemisphere for now:
        if alt >0: # Upper hemisphere
            # we assume that the device is oriented so that the servo
            # 0 position points north, and angle increases
            # clockwise. This means, that as long as 0<=az<180, we
            # need to do nothing:
            if az <180:
                az2 = az
                alt2 = alt
            else:
                # otherwise, we'll instead point at az - 180, and choose
                # 180 - alt (0 alt becomes 180 alt, 90 stays same)
                az2 = az - 180
                alt2 = 180 - alt
        else: # lower hemisphere
            # OK, here is a dirty trick to point to the lower
            # hemisphere as well:
            #
            # The pointer attached to the alt servo is a stick, so it
            # has two ends. So we can add a led to each end, and light
            # the end that is supposed to be pointing end.
            #
            # For now, I just add one led and light that if the moon
            # is up.
            if az < 180:
                # az stays same, but for alt: -1 -> 179, -90 -> 90)
                az2 = az
                alt2 = alt + 180
            else:
                # az: 180 -> 0, 360 -> 180)
                # alt: -1 -> +1, -90 ->90
                az2 = az - 180
                alt2 = - alt

        if UNIX != 1:
            # Blinkenlights
            tm.leds(1<<(int(t)%8)) # just make leds scroll every second.

            # for 4 seconds show AZ/ALT
            x = az
            y = alt
            if int(t)%8 < 4:
                pass
            elif int(t)%8 < 6:
                # show position for 2 seconds
                x = int(latitude)
                y = int(longitude)
            else:
                # show daet/time for 2 seconds
                x=mydate()
                y=mytime()
            tm_str="{:=4d}{:=4d}".format(x,y)
            tm.show(tm_str)
                
            # turn on led at pointer if moon is in upper hemisphere
            if alt >= 0:
                intensity = 1
            else:
                intensity = 0
            #np[0] = (int(255*intensity),int(25*intensity),int(25*intensity)) # color ?
            #np.write()

            
            # Update Servos
            servo_az.duty(az_angle_to_duty(az2))
            servo_alt.duty(alt_angle_to_duty(alt2))

        await asyncio.sleep(delay_ms/1000.0)
        if DEBUG:
            t = t + delay_ms/1000.0 * FAST_FORWARD_FACTOR
        else:
            t = time.time()
        

#async def updatetime(delay_sec):
#    while True:
#        ntptime.settime()
#        await asyncio.sleep(delay_sec)

async def getGPSFix(delay_sec):
    global latitude, longitude
    while True:
        if uart.any():
            while uart.any():
                r = uart.read()
                for c in r: mygps.update(chr(c))
            latitude = mygps.latitude[0]
            if mygps.latitude[1] == 'S':
                latitude = -latitude
            longitude = mygps.longitude[0]
            if mygps.longitude[1] == 'W':
                longitude = -longitude
            d=mygps.date
            t=mygps.timestamp
            # documentation is lying, parameters to pass are
            # Y,M,D,weekday,H,m,s,subsec
            # rather than
            # Y, M, D, H,m,s, weekday,subsec
            timetuple=(2000+d[2],d[1],d[0],0,t[0],t[1],int(t[2]),0)
            RTC().datetime(timetuple)

            if DEBUG:
                print("set lat/long to {} - {}, time is {}, was {}".format(latitude, longitude, timetuple, RTC().datetime()))
        await asyncio.sleep(delay_sec)


loop = asyncio.get_event_loop()

loop.create_task(asyncio.start_server(serve, "0.0.0.0", 8888))
loop.create_task(tracker(1000))
if MICROPYTHON == 1 and UNIX == 0:
    loop.create_task(getGPSFix(30))

loop.run_forever()
loop.close()

