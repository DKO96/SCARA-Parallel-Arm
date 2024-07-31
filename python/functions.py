import numpy as np
import matplotlib.pyplot as plt

l1 = 120
d = 118

def rad2Motor(r):
  return int((r * 4096) / (2*np.pi))

def motor2Rad(p):
  return (p / 4096) * 2*np.pi

def line(start, end, N):
  xl = np.linspace(start[0], end[0], N)
  yl = np.linspace(start[1], end[1], N)
  return np.vstack((xl, yl)).T

def circle(xc, yc, radius, N):
  angles = np.linspace(0, 2*np.pi, N, endpoint=False)
  x_coords = xc + radius * np.cos(angles)
  y_coords = yc + radius * np.sin(angles)
  return np.vstack((x_coords, y_coords)).T

def plotScara(n, ax, point, q1, q2):
  baseB = np.array([0, 0])
  baseA = np.array([d, 0])

  jointB = np.array([l1*np.cos(q1), l1*np.sin(q1)])
  jointA = np.array([d + l1*np.cos(q2), l1*np.sin(q2)])
  
  upperB = np.linalg.norm(jointB - baseB)
  upperA = np.linalg.norm(jointA - baseA)
  lowerB = np.linalg.norm(point - jointB)
  lowerA = np.linalg.norm(point - jointA)
  print(f"plot: {n}, upperB: {upperB:.2f}, upperA: {upperA:.2f}, "
                   f"lowerB: {lowerB:.2f}, lowerA: {lowerA:.2f}")

  ax.plot([baseB[0], jointB[0]], [baseB[1], jointB[1]], 'bo-', label="Arm 1")
  ax.plot([baseA[0], jointA[0]], [baseA[1], jointA[1]], 'ro-', label="Arm 2")
  ax.plot([jointB[0], point[0]], [jointB[1], point[1]], 'bo-')
  ax.plot([jointA[0], point[0]], [jointA[1], point[1]], 'ro-')
  ax.plot(point[0], point[1], 'go', label="End Effector")
  ax.set_title(f'SCARA Robot Configuration {n}')
  ax.set_xlabel('x-axis')
  ax.set_ylabel('y-axis')
  ax.grid(True)
  ax.legend()

def simulateScara(sleep, point, q1, q2):
  baseB = np.array([0, 0])
  baseA = np.array([d, 0])

  jointB = np.array([l1*np.cos(q1), l1*np.sin(q1)])
  jointA = np.array([d + l1*np.cos(q2), l1*np.sin(q2)])

  plt.cla()
  plt.plot([baseB[0], jointB[0]], [baseB[1], jointB[1]], 'bo-', label="Arm 1")
  plt.plot([baseA[0], jointA[0]], [baseA[1], jointA[1]], 'ro-', label="Arm 2")
  plt.plot([jointB[0], point[0]], [jointB[1], point[1]], 'bo-')
  plt.plot([jointA[0], point[0]], [jointA[1], point[1]], 'ro-')
  plt.plot(point[0], point[1], 'go', label="End Effector")
  plt.title(f'SCARA Robot Configuration')
  plt.xlabel('x-axis')
  plt.ylabel('y-axis')
  plt.grid(True)
  plt.xlim(-100, 250)
  plt.ylim(-10, 300)
  plt.legend()
  plt.draw()
  plt.pause(sleep)
