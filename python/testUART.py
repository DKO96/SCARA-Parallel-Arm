import time
import numpy as np
import serial 

def main():
  # initialize serial
  ser = serial.Serial('/dev/ttyUSB0', 38400)

  array = np.array([-1, 0, -1, 0, -1, 1, 1, 0, 1])

  mapping = {-1:b'01', 0:b'00', 1:b'10'}

  byte_array = b'\xBE' + b''.join(mapping[x] for x in array) + b'\xEF'

  print(byte_array)

  ser.write(byte_array)

  ser.close()


if __name__ == "__main__":
  main()
