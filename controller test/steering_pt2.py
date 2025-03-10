import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# parameters of pt2 system

# acc
#a = 0.0182
#b = 0.1796

# steering
a = 0.3969
b = 0.2394

# simulation parameters
T_s = 0.01

N = int (10 / T_s) # 10 seconds

# initialize input u and output y
u = np.ones(N)  # step input
y = np.zeros(N) 

# convert continuous-time parameters to discrete-time parameters in difference equation
A = a
B = -(b * T_s - 2 * a)
C = -(a - b * T_s + T_s**2)
D = T_s**2

# estimate the output use difference equation
for k in range(2, N):
    y[k] = (D * u[k] + B * y[k-1] + C * y[k-2]) / A

# compare result of discrete-time with continuous-time
G_s = ctl.TransferFunction([1], [a, b, 1])

# step response of continuous-time system
t_c, y_c = ctl.step_response(G_s)

# verify the result of discrete-time with continuous-time
plt.figure()
plt.plot(np.arange(N) * T_s, u, 'r--', label='Desired output')
plt.plot(np.arange(N) * T_s, y, 'bo-', label='Estimated output use difference equation')
plt.plot(t_c, y_c, 'r-', label='Estimated output use continuous-time system')
plt.xlabel('Time [s]')
plt.ylabel('steering angle [rad]')
plt.title('Step Response of PT2 steering system')
plt.legend()
plt.grid(True)
plt.show()
