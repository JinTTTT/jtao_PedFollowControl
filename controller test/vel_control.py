import numpy as np
import matplotlib.pyplot as plt

class VelocityControlTest:
    def __init__(self, a, b):
        self.Kp_vel = 0.02
        self.Ki_vel = 0.001
        self.Kd_vel = 0.8

        self.dt = 0.1

        self.v_max = 2.0
        self.acc_max = 2.0

        self.target_velocity = 2.0  # desired velocity [m/s]

        self.reference_velocity = 0.0  # reference velocity [m/s]
        self.vehicle_v_actual = 0.0  # actual velocity [m/s]

        self.last_error = 0.0  # error at last time step
        self.integral_error = 0.0  # integral error

        self.velocity_history = []  # actual velocity history for plotting
        self.reference_velocity_history = []  # reference velocity history for plotting
        self.no_control_velocity_history = []  # no control velocity history for plotting

        # pt2 system parameters
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

    def velocity_control(self):
        velocity_error = self.target_velocity - self.vehicle_v_actual

        velocity_error_dot = velocity_error - self.last_error
        self.integral_error += velocity_error

        # PID controller
        p_term = self.Kp_vel * velocity_error
        i_term = self.Ki_vel * self.integral_error
        d_term = self.Kd_vel * velocity_error_dot

        # reset integral error when velocity error is small
        if abs(velocity_error) < 0.1:
            self.integral_error = 0.0

        acceleration_command = p_term + d_term + i_term
        self.last_error = velocity_error

        # limit acceleration
        acceleration_command = np.clip(acceleration_command, -self.acc_max * self.dt, self.acc_max * self.dt)

        reference_velocity = self.reference_velocity + acceleration_command

        # limit reference velocity
        self.reference_velocity = np.clip(reference_velocity, 0, self.v_max)
        self.reference_velocity_history.append(self.reference_velocity)
        return self.reference_velocity

    def run_simulation(self, steps):
        for _ in range(steps):
            ref_vel = self.velocity_control()
            self.vehicle_v_actual = self.pt2_system_response_controlled(ref_vel)
            self.velocity_history.append(self.vehicle_v_actual)

            no_control_velocity = self.pt2_system_response_uncontrolled(self.target_velocity)
            self.no_control_velocity_history.append(no_control_velocity)

    def plot_velocity(self):
        plt.plot(self.velocity_history, label='actual velocity with PID control')
        plt.plot(self.no_control_velocity_history, label='actual velocity without PID control')
        #plt.plot(self.reference_velocity_history, label='reference velocity from PID control)')
        
        plt.plot([self.target_velocity] * len(self.velocity_history), 'r--', label='Desired Velocity')
        plt.xlabel('Time (0.1s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity Response with PID Control and PT2 System')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    # PT2 system parameters
    a = 0.3969
    b = 0.2394
    velocity_test = VelocityControlTest(a, b)

    steps = 200  # 20 seconds
    velocity_test.run_simulation(steps)
    velocity_test.plot_velocity()
