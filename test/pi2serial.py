# Pi to Serial Unit Tests
import serial, os

def check_serial(dev="/dev/ttyS0", baud=9600):
    try:
        s = serial.Serial(dev, baud)
        return True
    except:
        return False
