import numpy as np
import serial as ser

import inverseKin as ik
from simulation import Simulation

STEPSIZE = 0.003926991

def main():
  s = Simulation(120, 200, 118)

  # determine start and end coordinates
  start_coord = np.array([-41, 180])
  end_coord = np.array([159, 100])

  # create trajectory
  trajectory, angles = ik.lineTrajectory(start_coord, end_coord, s.L1, s.L2, s.D)

  steps = ik.stepAngles(angles, STEPSIZE)
  stepsA = steps[:,0]
  stepsB = steps[:,1]

  diffStepsA = np.diff(stepsA)
  diffStepsB = np.diff(stepsB)

  outputA = np.sign(diffStepsA)
  outputB = np.sign(diffStepsB)
  print(f"outputA: \n{len(outputA)}\n")
  print(f"outputB: \n{len(outputB)}\n")

  print(f"outputA: \n{outputA}\n")
  print(f"outputB: \n{outputB}\n")


if __name__ == "__main__":
  main()

