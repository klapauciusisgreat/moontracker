"""
Using the finish hardware setup for the moonpointer, we can use this
to calibrate the servos.

Switch 1/2 on the TM1638 increases/decreases AZ
Switrch 3/4 increases/decreases ALT
Switch 5 toggles auto-increase of AZ servo (10 degrees/sec)
switch 6 toggles aut-increase of ALT servo (10 degrees/sec)
"""

import machine
import tm1638

tm = tm1638.TM1638(stb=machine.Pin(4), clk=machine.Pin(16), dio=machine.Pin(17))
import time

servo_az = machine.PWM(machine.Pin(5), freq=50, duty=77)
servo_alt = machine.PWM(machine.Pin(3), freq=50, duty=77)


# True AZ/ALT:
az=0
alt=0

az_rotate = 0
alt_rotate = 0


# return PWM value to set Azimuth servo to specified angle.
# Empirically, roughly calibrated my specific servos
# Tower PRO SG90
def angle_to_duty(angle, a, b):
    if angle < 0 or angle > 180:
        print("illegal angle: ", angle)
        return 0
    return a + int((180-angle)/180 * b)
        
def az_angle_to_duty(angle):
    return angle_to_duty(angle, 32, 140-30)


# Funny how this hte same model servo but needs quite different calibration.
def alt_angle_to_duty(angle):
    return angle_to_duty(angle, 32, 121-32)

while True:
    servo_az.duty(az_angle_to_duty(az))
    servo_alt.duty(alt_angle_to_duty(alt))

    
    tm_str="{:=4d}{:=4d}".format(az, alt)
    tm.show(tm_str) #az #alt

    keys=tm.keys()
    # to keep it simple, only press one key at a time :)
    if keys & 1:
        az = (az + 1) % 180
    if keys & 2:
        az = (az - 1) % 180
    if keys & 4:
        alt = (alt + 1) % 180
    if keys & 8:
        alt = (alt - 1) % 180

    # toggle auto-rotation (only forward)
    
    if keys & 16:
        az_rotate = az_rotate ^ 1
    if keys & 32:
        alt_rotate = alt_rotate ^ 1

    if az_rotate:
        az = (az + 1) % 180
    if alt_rotate:
        alt = (alt + 1) % 180
 
    time.sleep(0.1) # 10 iterations per second
