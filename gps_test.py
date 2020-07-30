# Simple GPS module demonstration.
"""
This is a test program for the GOOUUU tech GT-U7 GPS board.


I'm using the micropyGPS library to receive updates.  Unfortunately,
the module, while getting fixes and sending time and locations, does
not seem to answer NMEA commands.

"""
from machine import UART, Pin
import utime as time

from micropyGPS import MicropyGPS

uart = UART(1, baudrate=9600)
uart.init(rx=21, tx=18)

mygps = MicropyGPS()

def send_command(command, add_checksum=True):
    """Send a command string to the GPS.  If add_checksum is True (the
    default) a NMEA checksum will automatically be computed and added.
    Note you should NOT add the leading $ and trailing * to the command
    as they will automatically be added!
    """
    uart.write('$')
    uart.write(command)
    print("sending: $" + command, end='')
    if add_checksum:
        checksum = 0
        for char in command:
            checksum ^= ord(char)
        uart.write('*')
        uart.write('{:02x}'.format(checksum).upper())
        print("*{:02x}".format(checksum).upper(), end='')
    uart.write('\r\n')
    print()

# None of these commands seem to do anything. Maybe commands can only
#be sent over USB?

#send_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
send_command('PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
send_command('PMTK220,2000')
send_command('PMTK251,38400')

while True:
    if uart.any():
        while uart.any():
            r = uart.read()
            print(str(r), end=''),
            for c in r: mygps.update(chr(c))
        print("\n------------------------")
        print(mygps.latitude, mygps.longitude)
        print (" ", mygps.date, mygps.timestamp)
        print(" ", mygps.satellites_used, mygps.fix_type,
              mygps.hdop ,mygps.vdop , mygps.pdop)
