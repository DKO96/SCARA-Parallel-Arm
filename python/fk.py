import numpy as np
import serial

l1 = 120
l2 = 125
d = 118

def motor2Rad(p):
  return (p / 4096) * 2*np.pi

def forwardKinematics(p1, p2):
  q1 = motor2Rad(p1)
  q2 = motor2Rad(p2)

  A = 2*l2*l1*np.sin(q2) - 2*l1*l2*np.cos(q1)
  B = 2*l2*d - 2*l1*l2*np.cos(q1) + 2*l2*l1*np.cos(q2)
  C = l1*l1 - l2*l2 + l1*l1 + l2*l2 + d*d \
      - l1*l1*np.sin(q1)*np.sin(q2) \
      - 2*l1*d*np.cos(q1) \
      + 2*l1*d*np.cos(q2) \
      - 2*l1*l1*np.cos(q1)*np.cos(q2)

  discriminant = A*A + B*B - C*C
  #print(f"discriminant {discriminant}")

  theta2_p = 2*np.arctan2(A + np.sqrt(A*A + B*B - C*C), B-C)
  theta2_m = 2*np.arctan2(A - np.sqrt(A*A + B*B - C*C), B-C)
                     
  temp1 = l2*np.sin(theta2_p) + l1*np.sin(q2) - l1*np.sin(q1)
  theta1_p = np.arcsin(temp1 / l2)
  #theta1_p = np.arctan2(temp1, l2)

  temp2 = l2*np.sin(theta2_m) + l1*np.sin(q2) - l1*np.sin(q1)
  theta1_m = np.arcsin(temp2 / l2)
  #theta1_m = np.arctan2(temp2, l2)

  #print(f"theta2_p: {theta2_p} theta2_m: {theta2_m} theta1_p: {theta1_p} theta1_m: {theta1_m}")

  xc_p = l1*np.cos(q1) + l2*np.cos(theta1_p)
  yc_p = l1*np.sin(q1) + l2*np.sin(theta1_p)

  xc_m = l1*np.cos(q1) + l2*np.cos(theta1_m)
  yc_m = l1*np.sin(q1) + l2*np.sin(theta1_m)

  print(f"xc_p: {xc_p} yc_p: {yc_p}")
  print(f"xc_m: {xc_m} yc_m: {yc_m}")
  print()


def main():
  ser = serial.Serial('/dev/ttyUSB0', baudrate=38400, timeout=1)

  while True:
    data = ser.readline().decode('utf-8').strip()

    if data:
      split_data = data.split('\t')

      values = {}
      for part in split_data:
        key, value = part.split(':')
        values[key.strip()] = int(value.strip())

      A_value = values.get('A', None)
      B_value = values.get('B', None)

      print(f"A: {A_value}, B: {B_value}")

      forwardKinematics(B_value, A_value)


if __name__ == "__main__":
  main()
