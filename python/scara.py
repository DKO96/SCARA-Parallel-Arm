import numpy as np
import matplotlib.pyplot as plt
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
    plt.figure()
    
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

      if wp.pen == 'down':
        bytesC = np.full(np.shape(bytesA), b'\xC1')
        plt.plot(path[:,0], path[:,1], 'b-', linewidth=2)
      else:
        bytesC = np.full(np.shape(bytesA), b'\xC0')
        plt.plot(path[:,0], path[:,1], 'r-', linewidth=2, alpha=0.4)

      commands = np.vstack((bytesA, bytesB, bytesC)).T
      trajectory.append(commands)

    plt.show()

    return np.vstack(trajectory)


####################################################################################################
##### Examples
####################################################################################################
def deathlyHallows():
  waypoints = [
    Waypoint(label='line', pen='down', start=[59, 310], end=[-41, 138]),
    Waypoint(label='line', pen='down', start=[-41, 138], end=[159, 138]),
    Waypoint(label='line', pen='down', start=[159, 138], end=[59, 310]),
    Waypoint(label='line', pen='up', start=[59, 310], end=[59, 138]),
    Waypoint(label='circle', pen='down', start=[59, 138], 
             radius=58, angle=3*np.pi/2, dir='ccw', arc=1),
  ]
  return Scara().getTrajectory(waypoints)

def horizontalLines():
  waypoints = [
    Waypoint(label='line', pen='up', start=[59, 89], end=[0, 100]),
    Waypoint(label='line', pen='down', start=[0, 100], end=[118, 100]),
    Waypoint(label='line', pen='up', start=[118, 100], end=[0, 110]),
    Waypoint(label='line', pen='down', start=[0, 110], end=[118, 110]),
    Waypoint(label='line', pen='up', start=[118, 110], end=[0, 120]),
    Waypoint(label='line', pen='down', start=[0, 120], end=[118, 120]),
    Waypoint(label='line', pen='up', start=[118, 120], end=[59, 89]),
  ]
  return Scara().getTrajectory(waypoints)

def helloWorld():
  waypoints = [
    Waypoint(label='line', pen='up', start=[59, 89], end=[120, 105]),

    # H
    Waypoint(label='line', pen='down', start=[120, 105], end=[120, 155]),
    Waypoint(label='line', pen='up', start=[120, 155], end=[120, 130]),
    Waypoint(label='line', pen='down', start=[120, 130], end=[90, 130]),
    Waypoint(label='line', pen='up', start=[90, 130], end=[90, 105]),
    Waypoint(label='line', pen='down', start=[90, 105], end=[90, 155]),
    Waypoint(label='line', pen='up', start=[90, 155], end=[80, 145]),

    # e
    Waypoint(label='line', pen='down', start=[80, 145], end=[60, 145]),
    #Waypoint(label='circle', pen='down', start=[60, 145], 
    #         radius=10, angle=np.pi, dir='ccw', arc=0.85),
    #Waypoint(label='line', pen='up', start=[64.12214748, 153.0901699], end=[50, 105]),
    Waypoint(label='circle', pen='down', start=[60, 145], 
             radius=10, angle=np.pi, dir='ccw', arc=0.90),
    Waypoint(label='line', pen='up', start=[61.90983006, 150.8778525], end=[50, 105]),

    # l
    Waypoint(label='line', pen='down', start=[50, 105], end=[50, 155]),
    Waypoint(label='line', pen='up', start=[50, 155], end=[40, 105]),

    # l 
    Waypoint(label='line', pen='down', start=[40, 105], end=[40, 155]),
    Waypoint(label='line', pen='up', start=[40, 155], end=[30, 145]),

    # o
    Waypoint(label='circle', pen='down', start=[30, 145], 
             radius=10, angle=0, dir='ccw', arc=1.1),
    Waypoint(label='line', pen='up', start=[30, 145], end=[90, 175]),

    # W
    Waypoint(label='line', pen='down', start=[90, 175], end=[80, 225]),
    Waypoint(label='line', pen='down', start=[80, 225], end=[70, 205]),
    Waypoint(label='line', pen='down', start=[70, 205], end=[60, 225]),
    Waypoint(label='line', pen='down', start=[60, 225], end=[50, 175]),
    Waypoint(label='line', pen='up', start=[50, 175], end=[40, 215]),

    # o
    Waypoint(label='circle', pen='down', start=[40, 215], 
             radius=10, angle=0, dir='ccw', arc=1),
    Waypoint(label='line', pen='up', start=[40, 215], end=[10, 205]),

    # r
    Waypoint(label='line', pen='down', start=[10, 205], end=[10, 225]),
    Waypoint(label='line', pen='up', start=[10, 225], end=[10, 215]),
    Waypoint(label='circle', pen='down', start=[10, 215], 
             radius=10, angle=0, dir='cw', arc=0.25),
    Waypoint(label='line', pen='up', start=[0, 205], end=[-10, 175]),

    # l
    Waypoint(label='line', pen='down', start=[-10, 175], end=[-10, 225]),
    Waypoint(label='line', pen='up', start=[-10, 225], end=[-40, 175]),

    # d
    Waypoint(label='line', pen='down', start=[-40, 175], end=[-40, 225]),
    Waypoint(label='line', pen='up', start=[-40, 225], end=[-40, 215]),
    Waypoint(label='circle', pen='down', start=[-40, 215], 
             radius=10, angle=np.pi, dir='cw', arc=1),

    # go home
    Waypoint(label='line', pen='up', start=[-40, 215], end=[59, 89]),
  ]
  return Scara().getTrajectory(waypoints)



