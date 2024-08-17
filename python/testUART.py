import time
import numpy as np
import serial 

def main():
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  mapping = {-1:b'01', 0:b'00', 1:b'10'}

  # motor A
  #array1 = np.full(600, -1)
  array1 = np.array([-1, 0, 1, 0])
  byteArray1 = b'\xBE' + b''.join(mapping[x] for x in array1)

  # motor B
  array2 = np.array([-1, 0, 1, 0])
  byteArray2 = b'\xC0' + b''.join(mapping[x] for x in array2) + b'\xDE'

  output = byteArray1 + byteArray2

  while True:
    print(f"Sending: {output}")
    ser.write(output)
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
