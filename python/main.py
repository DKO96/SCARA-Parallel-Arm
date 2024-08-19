import time
import numpy as np
import serial
import inverseKin as ik
from simulation import Simulation

STEPSIZE = 0.003926991

def main():
  s = Simulation(120, 200, 118)

  # determine start and end coordinates
  start_coord = np.array([59, 311])
  end_coord = np.array([-50, 200])

  # create trajectory
  trajectory, angles = ik.lineTrajectory(start_coord, end_coord, s.L1, s.L2, s.D)

  # get steps
  steps = ik.stepAngles(angles, STEPSIZE)
  stepsA = steps[:,0]
  stepsB = steps[:,1]

  outputA = np.sign(np.diff(stepsA)).astype(int)
  outputB = np.sign(np.diff(stepsB)).astype(int)

  print(f"outputA: \n{len(outputA)} \noutputB: \n{len(outputB)}")

  # map output
  mapping = {-1:b'\x01', 0:b'\x11', 1:b'\x10'}
  bytesA = np.array([mapping[x] for x in outputA])
  bytesB = np.array([mapping[x] for x in outputB])

  # initiate serial 
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  time.sleep(1)
  
  count = 0
  for i in range(len(outputA)):
    sending = b'\xBE' + bytesA[i] + b'\xC0' + bytesB[i] + b'\xDE'
    #print(sending)
    ser.write(sending)
    ser.flush()

    while ser.in_waiting:
      response = ser.readline().decode('ascii', errors='ignore').strip()
      if "N" in response:
        count += 1
        break
      time.sleep(0.01)

    time.sleep(0.01)


  print(count)

if __name__ == "__main__":
  main()
