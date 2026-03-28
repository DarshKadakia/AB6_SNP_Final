import serial
import struct
import teleop as teleop
import time

ser = serial.Serial('COM7', 115200)   # change port if needed

# struct format:
# 3f 6B 2i 7f 2f B
fmt = '<3f4B2i9fB4f'   # little endian---'<3f6B2i7f2fB'

size = struct.calcsize(fmt)

while True:
    # ---- Receive sensor struct ----
    teleop.update()
    if ser.read(1) == b'\xAA':
        data = ser.read(size)
        values = struct.unpack(fmt, data)

        print(values)
        # time.sleep(2)
    
    vel_packet = struct.pack('<B3f', 0xAA, teleop.vx, teleop.vy, teleop.vw)
    ser.write(vel_packet)