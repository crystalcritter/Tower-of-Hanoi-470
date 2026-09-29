import numpy as np
from scipy.linalg import logm
import math

# Link 1 position: [-0.15   0.15   0.162]
# Link 2 position: [-0.15   0.27   0.162]
# Link 3 position: [0.094 0.27  0.162]
# Link 4 position: [0.307 0.177 0.162]
# Link 5 position: [0.307 0.281 0.162]
# Link 6 position: [0.392 0.281 0.162]

omega1 = np.array([0, 0, 1])
q1 = np.array([-0.150, 0.150, 0.162])

omega2 = np.array([0, 1, 0])
q2 = np.array([-0.150, 0.270, 0.162])

omega3 = np.array([0, 1, 0])
q3 = np.array([0.094, 0.270, 0.162])

omega4 = np.array([0, 1, 0])
q4 = np.array([0.307, 0.177, 0.162])

omega5 = np.array([1, 0, 0])
q5 = np.array([0.307, 0.281, 0.162])

omega6 = np.array([0, 1, 0])
q6 = np.array([0.392, 0.281, 0.162])

def cal(omega, q):
    return -np.cross(omega, q)

print(cal(omega1, q1))
print(cal(omega2, q2))
print(cal(omega3, q3))
print(cal(omega4, q4))
print(cal(omega5, q5))
print(cal(omega6, q6))
