import numpy as np
import math
import matplotlib.pyplot as plt

class SteeringControlTest:
    def __init__(self, a, b):

        self.kp_steering = 1.0
        self.ki_steering = 2.0
        self.kd_steering = 0.5

        self.dt = 0.1

        self.steering_range_max = math.radians(30)                     # in [rad]
        self.steering_vel_max = math.radians(15)                       # in [rad/s]

        self.target_steering = math.radians(30)                     # desired steering angle [rad]

        self.reference_steering = 0                      # reference steering angle [rad]
        self.vehicle_steering_actual = 0                 # actual steering angle [rad]

        self.steering_last_error = 0                              # error at last time step
        self.steering_integral_error = 0                           # integral error

        self.steering_history = []                                  # actual steering history for plotting
        self.reference_steering_history = []                       # reference steering history for plotting
        self.no_control_steering_history = []  # no control steering history for plotting

        # pt2 steering system parameters
        # Y(s) = 1 / (as^2 + bs + 1)

        self.a = a
        self.b = b

        self.A = self.a
        self.B = -(self.b * self.dt - 2 * self.a)
        self.C = -(self.a - self.b * self.dt + self.dt ** 2)
        self.D = self.dt ** 2

        # initialize input and output for controlled system
        self.input_controlled = np.ones(2)
        self.output_controlled = np.zeros(2)

        # initialize input and output for uncontrolled system
        self.input_uncontrolled = np.ones(2)
        self.output_uncontrolled = np.zeros(2)

    def pt2_system_response_controlled(self, input_u):
        self.input_controlled = np.append(self.input_controlled, input_u)
        y_new = (self.D * self.input_controlled[-1] + self.B * self.output_controlled[-1] + self.C * self.output_controlled[-2]) / self.A
        self.output_controlled = np.append(self.output_controlled, y_new)
        return y_new

    def pt2_system_response_uncontrolled(self, input_u):
        self.input_uncontrolled = np.append(self.input_uncontrolled, input_u)
        y_new = (self.D * self.input_uncontrolled[-1] + self.B * self.output_uncontrolled[-1] + self.C * self.output_uncontrolled[-2]) / self.A
        self.output_uncontrolled = np.append(self.output_uncontrolled, y_new)
        return y_new

    def steering_control(self):

        steering_error = self.target_steering - self.vehicle_steering_actual
        steering_error_dot = steering_error - self.steering_last_error
        self.steering_integral_error += steering_error

        # reset integral error when steering error is small
        if abs(steering_error) < 0.01:
            self.steering_integral_error = 0.0

        # PID controller
        p_term = self.kp_steering * steering_error
        i_term = self.ki_steering * self.steering_integral_error
        d_term = self.kd_steering * steering_error_dot

        steering_change = p_term + i_term + d_term

        self.steering_last_error = steering_error

        # limit steering change
        max_steering_change = self.steering_vel_max * self.dt
        steering_change = np.clip(steering_change, -max_steering_change, max_steering_change)

        # update steering
        new_steering = self.vehicle_steering_actual + steering_change
        new_steering = np.clip(new_steering, -self.steering_range_max, self.steering_range_max)

        return new_steering

    def run_simulation(self, steps):
        # Reset input and output for uncontrolled system
        self.input_uncontrolled = np.ones(2)
        self.output_uncontrolled = np.zeros(2)

        for i in range(steps):

            ref_steering = self.steering_control()
            self.vehicle_steering_actual = self.pt2_system_response_controlled(ref_steering)
            self.reference_steering_history.append(ref_steering)
            self.steering_history.append(self.vehicle_steering_actual)

            # Simulate no control response
            if i == 0:
                
                no_control_steering = 0.0
            else:
                no_control_steering = self.pt2_system_response_uncontrolled(self.target_steering)
            self.no_control_steering_history.append(no_control_steering)

    def plot_steering(self):
        plt.figure()
        plt.plot(np.rad2deg(self.target_steering * np.ones(len(self.steering_history))), 'r--', label='Desired Steering [deg]')
        plt.plot(np.rad2deg(self.steering_history), 'g-', label='actual steering with PID control')
        #plt.plot(np.rad2deg(self.reference_steering_history), 'b-', label='Reference Steering [deg]')
        plt.plot(np.rad2deg(self.no_control_steering_history), 'm-', label='actual steering without PID control')
        plt.xlabel('Time [0.1 s]')
        plt.ylabel('Steering [deg]')
        plt.title('Steering Response with PID Control and PT2 System')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    a = 0.0182
    b = 0.1796

    steps = 50  # 5 seconds
    steering_control_test = SteeringControlTest(a, b)
    steering_control_test.run_simulation(steps)
    steering_control_test.plot_steering()
