import numpy as np

L1 = 120
L2 = 200
D = 118

def line(start, end, N):
  xl = np.linspace(start[0], end[0], N)
  yl = np.linspace(start[1], end[1], N)
  return np.vstack((xl, yl)).T

def inverseKinematics(x, y):
  r1 = np.sqrt(x**2 + y**2)
  tempQ1 = (L1**2 + r1**2 - L2**2) / (2*L1*r1)
  q1_1 = np.atan2(y, x) + np.acos(tempQ1)
  q1_2 = np.atan2(y, x) - np.acos(tempQ1)

  r2 = np.sqrt((D-x)**2 + y**2)
  tempQ2 = (L1**2 + r2**2 - L2**2) / (2*L1*r2)
  q2_1 = np.pi - np.atan2(y, D-x) - np.acos(tempQ2)
  q2_2 = np.pi - np.atan2(y, D-x) + np.acos(tempQ2)

  point = np.array([x[-1], y[-1]])

  return q1_1, q2_1

def getMotor(r):
  # convert rads to motor angles
  motorAngles =  np.floor((r * 4096) / (2*np.pi)).astype(int)
  wrappedAngles = (motorAngles % 4096).astype(int)
  return wrappedAngles

def printAngle(array):
  array_str = np.array2string(array, separator=',', formatter={'all':lambda x: f"{int(x):d}"})
  array_str = array_str.replace('[', '').replace(']', '')  # Optionally remove brackets
  print(array_str)

def main():
  # line
  start = np.array([0, 200])
  end   = np.array([120, 200])
  N = 100
  
  lr = line(start, end, N)
  rl = line(end, start, N)
  steps = np.vstack((lr, rl))

  # perform inverse kinematics
  q1, q2 = inverseKinematics(steps[:, 0], steps[:, 1])
  rads = np.vstack((q1, q2)).T
  motor = getMotor(rads)


  printAngle(motor[:,0])
  printAngle(motor[:,1])


if __name__ == "__main__":
  main();

