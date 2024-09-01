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
  penState = 'up'

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
      if wp.pen != Scara.penState:
        if wp.pen == 'down':
          pause = np.full((50, 3), b'\x11', dtype='object')
          pause[:, 2] = b'\xC0'
        else:
          pause = np.full((50, 3), b'\x11', dtype='object')
          pause[:, 2] = b'\xC1'

        trajectory.append(pause)

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
        Scara.penState = 'down'
      else:
        bytesC = np.full(np.shape(bytesA), b'\xC0')
        plt.plot(path[:,0], path[:,1], 'r-', linewidth=2, alpha=0.4)
        Scara.penState = 'up'

      commands = np.vstack((bytesA, bytesB, bytesC)).T
      trajectory.append(commands)

    #plt.show()

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

def circles():
  waypoints = [
    Waypoint(label='line', pen='up', start=[59, 89], end=[0, 150]),
    Waypoint(label='circle', pen='down', start=[0, 150], 
             radius=20, angle=0, dir='ccw', arc=1.1),
    Waypoint(label='line', pen='up', start=[0, 150], end=[59, 89]),
  ]
  return Scara().getTrajectory(waypoints)

def helloWorld():
  waypoints = [
    Waypoint(label='line', pen='up', start=[59, 89], end=[180, 110]),
    #Waypoint(label='line', pen='up', start=[59, 89], end=[50, 110]),

    # H
    Waypoint(label='line', pen='down', start=[180, 110], end=[180, 160]),
    Waypoint(label='line', pen='up', start=[180, 160], end=[180, 135]),
    Waypoint(label='line', pen='down', start=[180, 135], end=[150, 135]),
    Waypoint(label='line', pen='up', start=[150, 135], end=[150, 110]),
    Waypoint(label='line', pen='down', start=[150, 110], end=[150, 160]),
    Waypoint(label='line', pen='up', start=[150, 160], end=[140, 150]),

    # e
    Waypoint(label='line', pen='down', start=[140, 150], end=[120, 150]),
    Waypoint(label='circle', pen='down', start=[120, 150], 
             radius=10, angle=np.pi, dir='ccw', arc=0.90),
    Waypoint(label='line', pen='up', start=[121.9098301, 155.8778525], end=[110, 110]),

    # l
    Waypoint(label='line', pen='down', start=[110, 110], end=[110, 160]),
    Waypoint(label='line', pen='up', start=[110, 160], end=[100, 110]),

    # l 
    Waypoint(label='line', pen='down', start=[100, 110], end=[100, 160]),
    Waypoint(label='line', pen='up', start=[100, 160], end=[90, 150]),

    # o
    Waypoint(label='circle', pen='down', start=[90, 150], 
             radius=10, angle=0, dir='ccw', arc=1.1),
    Waypoint(label='line', pen='up', start=[90, 150], end=[50, 110]),

    # W
    Waypoint(label='line', pen='down', start=[50, 110], end=[40, 160]),
    Waypoint(label='line', pen='down', start=[40, 160], end=[30, 135]),
    Waypoint(label='line', pen='down', start=[30, 135], end=[20, 160]),
    Waypoint(label='line', pen='down', start=[20, 160], end=[10, 110]),
    Waypoint(label='line', pen='up', start=[10, 110], end=[0, 150]),

    # o
    Waypoint(label='circle', pen='down', start=[0, 150], 
             radius=10, angle=0, dir='ccw', arc=1.1),
    Waypoint(label='line', pen='up', start=[0, 150], end=[-30, 138]),

    # r
    Waypoint(label='line', pen='down', start=[-30, 138], end=[-30, 155]),
    Waypoint(label='line', pen='up', start=[-30, 155], end=[-28, 150]),
    Waypoint(label='circle', pen='down', start=[-28, 150], 
             radius=10, angle=0, dir='cw', arc=0.35),
    Waypoint(label='line', pen='up', start=[-43.87785252, 141.9098301], end=[-50, 110]),

    # l
    Waypoint(label='line', pen='down', start=[-50, 110], end=[-50, 155]),
    Waypoint(label='line', pen='up', start=[-50, 155], end=[-80, 110]),

    # d
    Waypoint(label='line', pen='down', start=[-80, 110], end=[-80, 155]),
    Waypoint(label='line', pen='up', start=[-80, 160], end=[-78, 150]),
    Waypoint(label='circle', pen='down', start=[-78, 150], 
             radius=10, angle=np.pi, dir='ccw', arc=1.1),

    # go home
    Waypoint(label='line', pen='up', start=[-78, 150], end=[59, 89]),
  ]
  return Scara().getTrajectory(waypoints)



