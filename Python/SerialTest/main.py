import sys

import serial
import time


def main():
    ser = serial.Serial()
    ser.port = 'COM4'
    ser.baudrate = 9600
    ser.timeout = .1
    ser.open()

    power = 0
    # Ramp motor 1 and motor 2 from -127 to 127(full reverse to full forward),
    # waiting 20ms(1 / 50th of a second) per value.
    for power in range(-127, 127):
        ser.write(power.to_bytes(1, sys.byteorder))
        ser.write(power.to_bytes(1, sys.byteorder))
        time.sleep(0.02)

    # now go back the way we came
    for power in range(127, -127, -1):
        ser.write(power.to_bytes(1, sys.byteorder))
        ser.write(power.to_bytes(1, sys.byteorder))
        time.sleep(0.02)

    ser.close()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
