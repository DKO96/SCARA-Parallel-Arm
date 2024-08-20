import time
import numpy as np
import serial
import inverseKin as ik
from simulation import Simulation

class Scara:
  def __init__(self, L1, L2, D, stepsize):
    self.L1 = L1
    self.L2 = L2
    self.D = D
    self.stepsize = stepsize
    self.mapping = {-1:b'\x01', 0:b'\x11', 1:b'\x10'}

  def drawLine(self, waypoints):
    trajectory = []
    for i in range(1, len(waypoints)):
      # create trajectory
      path, angles = ik.lineTrajectory(waypoints[i - 1], waypoints[i], self.L1, self.L2, self.D)

      # get steps
      steps = ik.stepAngles(angles, self.stepsize)
      stepsA = np.sign(np.diff(steps[:,0])).astype(int)
      stepsB = np.sign(np.diff(steps[:,1])).astype(int)

      # map output
      bytesA = np.array([self.mapping[x] for x in stepsA])
      bytesB = np.array([self.mapping[x] for x in stepsB])

      commands = np.vstack((bytesA, bytesB)).T
      trajectory.append(commands)

    return np.vstack(trajectory)

  def drawCircle(self, trajectory, start, r, angle):
    circle, angles = ik.circleTrajectory(start, r, angle, self.L1, self.L2, self.D)

    # get steps
    steps = ik.stepAngles(angles, self.stepsize)
    stepsA = np.sign(np.diff(steps[:,0])).astype(int)
    stepsB = np.sign(np.diff(steps[:,1])).astype(int)

    # map output
    bytesA = np.array([self.mapping[x] for x in stepsA])
    bytesB = np.array([self.mapping[x] for x in stepsB])

    commands = np.vstack((bytesA, bytesB)).T
    trajectory = np.vstack((trajectory, commands))

    return trajectory


def main():
  s = Scara(120, 200, 118, 0.003926991)

  # waypoint coordinates
  waypoints = np.array([
    [59, 311], 
    [-41, 138], 
    [159, 138],
    [59, 311], 
    [59, 138], 
  ])

  trajectory = s.drawLine(waypoints)

  start = np.array([59, 138])
  r = 58
  angle = 3*np.pi/2
  trajectory = s.drawCircle(trajectory, start, r, angle)

  # initiate serial 
  ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
  time.sleep(2)
  ser.reset_input_buffer()
  
  for i in range(len(trajectory)):
    sending = b'\xBE' + trajectory[i, 0] + b'\xC0' + trajectory[i, 1] + b'\xDE'
    ser.write(sending)
    ser.flush()

    while True:
      if ser.in_waiting:
        response = ser.readline().decode('ascii', errors='ignore').strip()
        if "N" in response:
          break


if __name__ == "__main__":
  main()
