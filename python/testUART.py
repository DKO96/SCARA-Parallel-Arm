import time
import numpy as np
import serial 

def main():
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  array = np.array([-1, 0, -1, 0, -1, 1, 1, 0, 1])
  mapping = {-1:b'01', 0:b'00', 1:b'10'}
  byte_array = b'\xBE' + b''.join(mapping[x] for x in array) + b'\xEF'

  while True:
    print(f"Sending: {byte_array}")
    ser.write(byte_array)
    ser.flush()

    time.sleep(0.5)

    # read response
    while ser.in_waiting:
      response = ser.readline().decode('ascii', errors='ignore').strip()
      print(f"Received: {response}")
      if "DONE" in response:
        break

    time.sleep(2)

  ser.close()


if __name__ == "__main__":
  main()
