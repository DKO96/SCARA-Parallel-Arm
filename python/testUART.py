import time
import numpy as np
import serial 

def main():
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  mapping = {-1:b'01', 0:b'00', 1:b'10'}
  step = 1
  n = 1

  # motor A
  array1 = np.full(n, step)
  #array1 = np.array([-1, 0, 1, 0])
  byteArray1 = b'\xBE' + b''.join(mapping[x] for x in array1)

  # motor B
  array2 = np.full(n, step)
  #array2 = np.array([-1, 0, 1, 0])
  byteArray2 = b'\xC0' + b''.join(mapping[x] for x in array2) + b'\xDE'

  output = byteArray1 + byteArray2

  while True:
    ser.write(output)
    ser.flush()

    # read response
    while ser.in_waiting:
      response = ser.readline().decode('ascii', errors='ignore').strip()
      if "DONE" in response:
        break


  ser.close()


if __name__ == "__main__":
  main()
