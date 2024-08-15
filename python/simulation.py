import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import inverseKin as ik

class Simulation:
  def __init__(self, L1, L2, D):
    self.L1 = L1
    self.L2 = L2
    self.D = D
    self.baseA = np.array([0, 0])
    self.baseB = np.array([D, 0])

  def plotScara(self, start, end):
    c = ["bo-", "go-"]
    p = [start, end]
    l = ["start", "end"]
    a = [0.3, 1]

    plt.figure(figsize=(12, 10))

    for i in range(len(p)):
      # get position of stepper arms
      jointA = np.array([self.L1*np.cos(p[i][0]), self.L1*np.sin(p[i][0])])
      jointB = np.array([self.D + self.L1*np.cos(p[i][1]), self.L1*np.sin(p[i][1])])

      # plot stepper arms
      plt.plot([self.baseA[0], jointA[0]], [self.baseA[1], jointA[1]], 
               c[i], label=l[i], linewidth=4, alpha=a[i])
      plt.plot([self.baseB[0], jointB[0]], [self.baseB[1], jointB[1]], 
               c[i], linewidth=4, alpha=a[i])
      
      # plot desired end-effector trajectory
      plt.plot([p[0][2][0], p[1][2][0]], [p[0][2][1], p[1][2][1]], 'r-', linewidth=2)

      # plot end-effector position
      plt.plot([jointA[0], p[i][2][0]], [jointA[1], p[i][2][1]], c[i], linewidth=4, alpha=a[i])
      plt.plot([jointB[0], p[i][2][0]], [jointB[1], p[i][2][1]], c[i], linewidth=4, alpha=a[i])

      plt.title(f'SCARA Robot Configuration')
      plt.xlabel('x-axis')
      plt.ylabel('y-axis')
      plt.xlim([-132, 250])
      plt.ylim([-61, 321])
      plt.grid(True)
      plt.legend()

    plt.show()

  def animation(self, traj, ang):
    plt.figure(figsize=(12, 10))

    for i in range(len(ang)):
      plt.clf()

      # get position of stepper arms
      jointA = np.array([self.L1*np.cos(ang[i][0]), self.L1*np.sin(ang[i][0])])
      jointB = np.array([self.D + self.L1*np.cos(ang[i][1]), self.L1*np.sin(ang[i][1])])

      # plot stepper arms
      plt.plot([self.baseA[0], jointA[0]], [self.baseA[1], jointA[1]], 'go-', linewidth=4)
      plt.plot([self.baseB[0], jointB[0]], [self.baseB[1], jointB[1]], 'bo-', linewidth=4)

      # plot desired trajectory
      plt.plot(traj[:,0], traj[:,1], 'r-')
      
      # plot possible end-effector position
      circleA = patches.Circle((jointA), 200, edgecolor='green', 
                               facecolor='none', linewidth=2, alpha=0.5)
      plt.gca().add_patch(circleA)

      circleB = patches.Circle((jointB), 200, edgecolor='blue', 
                               facecolor='none', linewidth=2, alpha=0.5)
      plt.gca().add_patch(circleB)

      # plot end-effector position
      #plt.plot([jointA[0], traj[i][0]], [jointA[1], traj[i][1]], 'go-', linewidth=4)
      #plt.plot([jointB[0], traj[i][0]], [jointB[1], traj[i][1]], 'bo-', linewidth=4)

      plt.title(f'SCARA Robot Configuration')
      plt.xlabel('x-axis')
      plt.ylabel('y-axis')
      plt.xlim([-142, 260])
      plt.ylim([-71, 331])
      plt.grid(True)

      plt.pause(0.001)

    plt.show()


def main():
  s = Simulation(120, 200, 118)

  start_coord = np.array([-41, 180])
  end_coord = np.array([159, 100])

  s1, s2 = ik.inverseKinematics(start_coord[0], start_coord[1], s.L1, s.L2, s.D)
  e1, e2 = ik.inverseKinematics(end_coord[0], end_coord[1], s.L1, s.L2, s.D)

  # create trajectory
  trajectory, angles = ik.lineTrajectory(start_coord, end_coord, s.L1, s.L2, s.D)

  # plot manipulator position
  start = (s1, s2, start_coord)
  end = (e1, e2, end_coord)
  s.plotScara(start, end)

  # run animation
  s.animation(trajectory, angles)


if __name__ == "__main__":
  main()
