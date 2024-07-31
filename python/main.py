import time
import numpy as np
import matplotlib.pyplot as plt
import serial
import struct

import functions as f

l1 = 120
l2 = 200
d = 118

def getMotor(rB, rA):
  # convert rads to motor angles
  B = f.rad2Motor(rB)
  A = f.rad2Motor(rA)

  # clamp motor angles
  B = max(150, min(2048, B)) 
  A = max(0, min(1750, A)) 

  return A, B

def inverseKinematics(x, y, prevA, prevB):
  r1 = np.sqrt(x**2 + y**2)
  tempQ1 = (l1**2 + r1**2 - l2**2) / (2*l1*r1)
  q1_1 = np.atan2(y, x) + np.acos(tempQ1)
  q1_2 = np.atan2(y, x) - np.acos(tempQ1)

  r2 = np.sqrt((d-x)**2 + y**2)
  tempQ2 = (l1**2 + r2**2 - l2**2) / (2*l1*r2)
  q2_1 = np.pi - np.atan2(y, d-x) - np.acos(tempQ2)
  q2_2 = np.pi - np.atan2(y, d-x) + np.acos(tempQ2)

  #print(f"q1_1(B): {q1_1[-1]:.4f}, q1_2(B): {q1_2[-1]:.4f}, "
  #      f"q2_1(A): {q2_1[-1]:.4f}, q2_2(A): {q2_2[-1]:.4f}, "
  #)

  point = np.array([x[-1], y[-1]])

  #fig, axs = plt.subplots(2, 2, figsize=(12, 10))
  #f.plotScara(1, axs[0,0], point, q1_1[-1], q2_1[-1])
  #f.plotScara(2, axs[0,1], point, q1_2[-1], q2_1[-1])
  #f.plotScara(3, axs[1,0], point, q1_1[-1], q2_2[-1])
  #f.plotScara(4, axs[1,1], point, q1_2[-1], q2_2[-1])
  #plt.tight_layout()
  #plt.show()

  return q1_1, q2_1


def main():
  # initialize serial port
  ser = serial.Serial('/dev/ttyUSB0', baudrate=38400, timeout=1)

  # track motor position
  while (True):
    current = ser.readline().decode('utf-8').strip()

    if current:
      split_current = current.split('\t')

      values = {}
      for part in split_current:
        try:
          key, value = part.split(':')
          values[key.strip()] = int(value.strip())
        except ValueError:
          continue

      curA = values.get('A', None)
      curB = values.get('B', None)
      
      if curA and curB:
        print(f"current A: {curA}, current B: {curB}")
        break

  # circle center
  xc = 59
  yc = 225 
  r = 60
  steps = f.circle(xc, yc, r, 8)
  print(f"circle steps:\n {steps}")

  # line
  h = 220
  start = np.array([0, h])
  end   = np.array([118, h])
  N = 550
  
  lr = f.line(start, end, N)
  rl = f.line(end, start, N)
  steps = np.vstack((lr, rl))
  print(f"line steps:\n {steps}")

  # perform inverse kinematics
  q1, q2 = inverseKinematics(steps[:, 0], steps[:, 1], curA, curB)
  rads = np.vstack((q1, q2)).T
  data = np.hstack((steps, rads))
  print(f"data:\n {data}")

  # convert angle to hexadecimal for USART
  motor_cmd = []
  for i in range(len(q1)):
    motor_cmd.append(struct.pack('>BHHB', 0xFF, *getMotor(q1[i], q2[i]), 0x00))

  # send data to microcontroller
  sleep = 1
  while False:
    for i in range(len(motor_cmd)):
      #ser.write(motor_cmd[i])
      print(f"point: {data[i,0:2]}, B: {f.rad2Motor(data[i,2])}, A: {f.rad2Motor(data[i,3])}")

      time.sleep(0.001)

      f.simulateScara(sleep, data[i,0:2], data[i,2], data[i,3])
    


if __name__ == "__main__":
  main()
