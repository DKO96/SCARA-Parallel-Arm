import numpy as np

L1 = 120
L2 = 200
D = 118

def inverseKinematics(x, y):
  r1 = np.sqrt(x**2 + y**2)
  theta = np.atan2(y, x)
  gamma = np.acos((r1**2 + L1**2 - L2**2) / (2*L1*r1))
  q1 = np.pi + gamma + theta

  r2 = np.sqrt((D - x)**2 + y**2)
  phi = np.atan2(y, (D - x))
  nu = np.acos((r2**2 + L1**2 - L2**2) / (2*L1*r2))
  q2 = np.pi + (np.pi - phi - nu)

  return q1, q2

def convertAngle(r):
  # convert rads to motor angles
  motorAngles =  np.floor((r * 4096) / (2*np.pi)).astype(int)
  wrappedAngles = (motorAngles % 4096).astype(int)
  return wrappedAngles

def main():
  x = 139
  y = 180

  q1, q2 = inverseKinematics(x, y)

  p1 = (convertAngle(q1))
  p2 = (convertAngle(q2))
  print(f"q1 = {q1}, p1 = {p1}")
  print(f"q2 = {q2}, p2 = {p2}")


if __name__ == "__main__":
  main();


