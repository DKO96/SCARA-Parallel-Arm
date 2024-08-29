import time
import numpy as np
import serial
import inverseKin as ik
from simulation import Simulation
import scara as ex

def main():
  # run example
  #trajectory = ex.deathlyHallows()
  #trajectory = ex.horizontalLines()
  trajectory = ex.helloWorld()

  # initiate serial 
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  time.sleep(2)
  ser.reset_input_buffer()
  
  for i in range(len(trajectory)):
    sending = b'\xBE' + trajectory[i, 0] + trajectory[i, 2] + trajectory[i, 1] + b'\xDE'
    ser.write(sending)
    ser.flush()

    while True:
      if ser.in_waiting:
        response = ser.readline().decode('ascii', errors='ignore').strip()
        if "N" in response:
          break

    time.sleep(0.005)


if __name__ == "__main__":
  main()
