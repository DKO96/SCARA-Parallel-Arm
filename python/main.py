import numpy as np

import inverseKin as ik
from simulation import Simulation

STEPSIZE = 0.003926991
TOLERANCE = 1e-9

def main():
  s = Simulation(120, 200, 118)

  # determine start and end coordinates
  start_coord = np.array([-41, 180])
  end_coord = np.array([159, 100])

  # create trajectory
  trajectory, angles = ik.lineTrajectory(start_coord, end_coord, s.L1, s.L2, s.D)

  motorA = angles[:,0]
  motorB = angles[:,1]
  #print(f"motorA: \n{motorA}\n")
  #print(f"motorB: \n{motorB}\n")

  steps = ik.stepAngles(angles, STEPSIZE)
  stepsA = steps[:,0]
  stepsB = steps[:,1]
  #print(f"stepsA: \n{stepsA}\n")
  #print(f"stepsB: \n{stepsB}\n")

  diffStepsA = np.diff(stepsA)
  diffStepsB = np.diff(stepsB)
  #print(f"diffStepsA: \n{diffStepsA}\n")
  #print(f"diffStepsB: \n{diffStepsB}\n")

  outputA = np.sign(diffStepsA)
  outputB = np.sign(diffStepsB)
  print(f"outputA: \n{len(outputA)}\n")
  print(f"outputB: \n{len(outputB)}\n")

  s.animation(trajectory, steps)

if __name__ == "__main__":
  main()

