import numpy as np
import inverseKin as ik

class Waypoint:
  def __init__(self, label, pen, start, end=None, 
               radius=None, angle=None, dir=None, arc=None):
    self.label = label
    self.pen = pen
    self.start = start
    self.end = end
    self.radius = radius
    self.angle = angle
    self.dir = dir
    self.arc = arc

class Scara:
  def __init__(self):
    self.L1 = 120
    self.L2 = 200
    self.D = 118
    self.stepsize = 0.003926991
    self.mapping = {-1:b'\x01', 0:b'\x11', 1:b'\x10'}

  def getTrajectory(self, waypoints):
    trajectory = []
    
    for wp in waypoints:
      if wp.label == 'line':
        path, angles = ik.lineTrajectory(wp.start, wp.end, self.L1, self.L2, self.D)
      elif wp.label == 'circle':
        path, angles = ik.circleTrajectory(wp.start, wp.radius, wp.angle, wp.dir, wp.arc,
                                           self.L1, self.L2, self.D)
      
      steps = ik.stepAngles(angles, self.stepsize)
      stepsA = np.sign(np.diff(steps[:,0])).astype(int)
      stepsB = np.sign(np.diff(steps[:,1])).astype(int)

      bytesA = np.array([self.mapping[x] for x in stepsA])
      bytesB = np.array([self.mapping[x] for x in stepsB])

      commands = np.vstack((bytesA, bytesB)).T
      trajectory.append(commands)

    return np.vstack(trajectory)


####################################################################################################
##### Examples
####################################################################################################
def deathlyHallows():
  waypoints = [
    Waypoint(label='line', pen='down', start=[59, 310], end=[-41, 138]),
    Waypoint(label='line', pen='down', start=[-41, 138], end=[159, 138]),
    Waypoint(label='line', pen='down', start=[159, 138], end=[59, 310]),
    Waypoint(label='line', pen='down', start=[59, 310], end=[59, 138]),
    Waypoint(label='circle', pen='down', start=[59, 138], 
             radius=58, angle=3*np.pi/2, dir='ccw', arc=1),
  ]
  return Scara().getTrajectory(waypoints)




