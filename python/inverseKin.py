import numpy as np

def inverseKinematics(x, y, L1, L2, D):
  r1 = np.sqrt(x**2 + y**2)
  theta = np.atan2(y, x)
  gamma = np.acos((r1**2 + L1**2 - L2**2) / (2*L1*r1))
  q1 = gamma + theta

  r2 = np.sqrt((D - x)**2 + y**2)
  phi = np.atan2(y, (D - x))
  nu = np.acos((r2**2 + L1**2 - L2**2) / (2*L1*r2))
  q2 = np.pi - phi - nu

  return q1, q2

def lineTrajectory(start, end, L1, L2, D):
  N = int(np.linalg.norm(start - end))*4
  xl = np.linspace(start[0], end[0], N)
  yl = np.linspace(start[1], end[1], N)

  trajectory = np.vstack((xl, yl)).T

  q1, q2 = inverseKinematics(xl, yl, L1, L2, D)
  angles = np.vstack((q1, q2)).T

  return trajectory, angles

def convertAngle(r):
  # convert rads to motor angles
  motorAngles =  np.floor((r * 4096) / (2*np.pi)).astype(int)
  wrappedAngles = (motorAngles % 4096).astype(int)
  return wrappedAngles

def stepAngles(angles, stepsize):
  q1_adjusted = [angles[0, 0]]
  q2_adjusted = [angles[0, 1]]

  for i in range(1, len(angles)):
    q1 = q1_adjusted[-1]
    q2 = q2_adjusted[-1]

    while np.abs(angles[i, 0] - q1) > stepsize:
      if angles[i, 0] > q1:
        q1 += stepsize
      else:
        q1 -= stepsize
      q1_adjusted.append(q1)
      q2_adjusted.append(q2_adjusted[-1])

    while np.abs(angles[i, 1] - q2) > stepsize:
      if angles[i, 1] > q2:
        q2 += stepsize
      else:
        q2 -= stepsize
      q1_adjusted.append(q1_adjusted[-1])
      q2_adjusted.append(q2)

  return np.vstack((q1_adjusted, q2_adjusted)).T
