import math
import numpy as np
import matplotlib.pyplot as plt

class VehicleSimulator:
    def __init__(self, case_number=1):
        # 车辆参数
        self.wheelbase = 2.9  # 车轮基距 [m]
        self.v_max = 2.0  # 最大速度 [m/s]
        self.acc_max_forward = 0.5  # 最大加速度 [m/s^2]
        self.acc_max_deceleration = 0.5  # 最大减速度 [m/s²]
        self.steering_range_max = 24 * math.pi / 180  # 最大转向角 [rad]
        self.steering_change_max = np.deg2rad(11.4)  # 最大转向角速度 [rad/s]
        
        # 控制目标
        self.desired_distance = 5.0  # 期望与行人的距离 [m]
        
        # 控制器状态变量
        self.target_vel_dist_control_last = 0.0
        self.desired_steering_latest = 0.0
        
        # 仿真参数
        self.dt = 0.1  # 时间步长 [s]
        self.simulation_time = 50.0  # 仿真总时间 [s]
        
        # 记录数据
        self.time_history = []
        self.vehicle_x_history = []
        self.vehicle_y_history = []
        self.vehicle_yaw_history = []
        self.vehicle_v_history = []
        self.vehicle_acc_history = []
        self.vehicle_steering_history = []
        self.pedestrian_x_history = []
        self.pedestrian_y_history = []
        self.pedestrian_v_history = []
        self.distance_history = []
        self.desired_speed_history = []
        self.desired_steering_history = []
        self.proportional_history = []
        self.derivative_history = []
        self.d_stop = []
        self.d_headway = []
        self.d_required = []
           
        # 测试用例选择
        self.case_number = case_number
        
        # 初始化车辆和行人状态
        self.initialize_states()
    
    def initialize_states(self):
        # 根据测试用例设置初始条件
        if self.case_number == 1:
            # 测试用例 1：静止的行人
            self.vehicle_x = 0.0
            self.vehicle_y = 0.0
            self.vehicle_yaw = 0.0
            self.vehicle_v_actual = 0.0
            self.vehicle_steering_actual = 0.0
            
            self.pedestrian_x = 20.0
            self.pedestrian_y = 0.0
            self.pedestrian_v = 0.0
            
        elif self.case_number == 2:
            # 测试用例 2：行人移动并在 10 秒后减速
            self.vehicle_x = 0.0
            self.vehicle_y = 0.0
            self.vehicle_yaw = 0.0
            self.vehicle_v_actual = 0.0
            self.vehicle_steering_actual = 0.0
            
            self.pedestrian_x = 5.0
            self.pedestrian_y = 0.0
            self.pedestrian_v = 0.5  # 初始速度为 0.5 m/s
            
        else:
            raise ValueError("Invalid case number. Please choose case_number=1 or 2.")
    
    def run_simulation(self):
        t = 0.0
        vehicle_v_actual_last = self.vehicle_v_actual
        while t < self.simulation_time:
            # 更新行人位置和速度
            self.update_pedestrian_state(t)
            
            # 计算相对位置和距离
            rel_ped_x, rel_ped_y, distance = self.calculate_relative_position()
            rel_ped_speed = self.pedestrian_v - self.vehicle_v_actual
            # 高级速度控制：距离控制
            target_vel_high_level = self.distance_control(
                distance, self.desired_distance, rel_ped_speed, self.dt,
                self.v_max, self.acc_max_forward, self.acc_max_deceleration,
                self.target_vel_dist_control_last, self.vehicle_v_actual)
            self.target_vel_dist_control_last = target_vel_high_level
            
            # 车辆速度直接设定为目标速度（假设完美执行）
            self.vehicle_v_actual = target_vel_high_level
            
            # 计算车辆加速度
            vehicle_acc = (self.vehicle_v_actual - vehicle_v_actual_last) / self.dt
            self.vehicle_acc_history.append(vehicle_acc)
            vehicle_v_actual_last = self.vehicle_v_actual
            
            # 横向控制：转向控制
            desired_steering_angle, _, _, _ = self.steering_control(
                rel_ped_x, rel_ped_y, self.vehicle_v_actual, self.desired_steering_latest,
                self.dt, self.steering_change_max, self.steering_range_max)
            self.desired_steering_latest = desired_steering_angle
            
            # 更新车辆状态
            self.update_vehicle_state(self.vehicle_v_actual, desired_steering_angle)
            
            # 记录数据
            self.record_data(t, distance, self.vehicle_v_actual, desired_steering_angle)
            
            # 时间更新
            t += self.dt
                    
        # 仿真结束后绘制结果
        self.plot_results()
    
    def update_pedestrian_state(self, t):
        # 更新行人速度和位置
        if self.case_number == 2:
            # 在 10 秒后，行人开始减速
            if t >= 20.0 and self.pedestrian_v > 0.0:
                deceleration = self.acc_max_forward
                self.pedestrian_v -= deceleration * self.dt
                self.pedestrian_v = max(self.pedestrian_v, 0.0)  # 确保速度不为负
        # 更新行人位置
        self.pedestrian_x += self.pedestrian_v * self.dt
        # 记录行人速度
        self.pedestrian_v_history.append(self.pedestrian_v)
    
    def calculate_relative_position(self):
        # 计算行人相对于车辆的全局坐标差
        dx = self.pedestrian_x - self.vehicle_x
        dy = self.pedestrian_y - self.vehicle_y
        
        # 将全局坐标差转换为车辆坐标系下的相对位置
        rel_ped_x = dx * math.cos(-self.vehicle_yaw) - dy * math.sin(-self.vehicle_yaw)
        rel_ped_y = dx * math.sin(-self.vehicle_yaw) + dy * math.cos(-self.vehicle_yaw)
        
        # 计算距离
        distance = math.hypot(rel_ped_x, rel_ped_y)
        
        return rel_ped_x, rel_ped_y, distance
    
    def distance_control(self, dist_to_ped_now, track_distance, relative_distance_rate,
                     cmd_dt, v_max, acc_max_forward, acc_max_deceleration, target_vel_dist_control_last,
                     vehicle_v_actual):
        
        # 时间头距
        t_headway = 1.5  # 根据安全要求调整
        d_headway = t_headway * vehicle_v_actual

        # 计算停止距离（前馈项）
        acc_stop = 0.5 * acc_max_deceleration
        d_stop = (vehicle_v_actual ** 2) / (2 * acc_stop)
        
        # 计算所需的安全距离，加入安全裕度
        safety_margin = 0.0  # 根据需要调整
        required_distance = max(d_stop, d_headway) + track_distance + safety_margin

        self.d_stop.append(d_stop)
        self.d_headway.append(d_headway)
        self.d_required.append(required_distance)

        # 计算误差和误差变化率
        error = dist_to_ped_now - required_distance
        error_rate = relative_distance_rate  # 相对速度

        # 控制器增益
        Kp = 0.5
        Kd = 0.6  # 根据需要调整微分增益

        # 计算比例和微分项
        proportional_contribution = Kp * error
        derivative_contribution = Kd * error_rate

        # 计算目标速度
        desired_target_vel = vehicle_v_actual + proportional_contribution + derivative_contribution


        # 限制目标速度在 0 到 v_max 之间
        desired_target_vel = np.clip(desired_target_vel, 0.0, v_max)

        # 计算速度增量
        delta_v = desired_target_vel - vehicle_v_actual

        # 应用加速度和减速度限制
        if delta_v > 0:
            # 加速
            delta_v_max = acc_max_forward * cmd_dt
            delta_v = min(delta_v, delta_v_max)
        else:
            # 减速
            delta_v_max = acc_max_deceleration * cmd_dt
            delta_v = max(delta_v, -delta_v_max)

        target_vel_dist_control = vehicle_v_actual + delta_v

        # 平滑速度变化
        d_vel = target_vel_dist_control - target_vel_dist_control_last
        if d_vel > 0:
            d_vel_max = acc_max_forward * cmd_dt
            d_vel = min(d_vel, d_vel_max)
        else:
            d_vel_max = acc_max_deceleration * cmd_dt
            d_vel = max(d_vel, -d_vel_max)

        target_vel_high_level_limited = target_vel_dist_control_last + d_vel
        target_vel_high_level_limited = np.clip(target_vel_high_level_limited, 0.0, v_max)

        # 记录比例和微分项的贡献（用于绘图）
        self.proportional_history.append(proportional_contribution)
        self.derivative_history.append(derivative_contribution)

        return target_vel_high_level_limited



    def steering_control(self, rel_ped_x, rel_ped_y, vehicle_v_actual, desired_steering_latest, 
                         cmd_dt, steering_change_max, steering_range_max):
        # 计算航向误差
        heading_error = math.atan2(rel_ped_y, rel_ped_x)
        # 计算横向误差
        cross_track_error = rel_ped_y
        # Stanley 控制器参数
        Ke = 0.05
        softening_constant = 1.0
        # 计算横向误差项
        cross_error = math.atan2(Ke * cross_track_error, vehicle_v_actual + softening_constant)
        # 计算转向角
        steering_angle_stanley = heading_error + cross_error
        # 限制转向角变化
        d_st_agl_max = steering_change_max * cmd_dt
        d_st_angle = steering_angle_stanley - desired_steering_latest
        d_st_angle = np.clip(d_st_angle, -d_st_agl_max, d_st_agl_max)
        # 更新期望转向角
        desired_steering_angle = desired_steering_latest + d_st_angle
        desired_steering_angle = np.clip(desired_steering_angle, -steering_range_max, steering_range_max)
        return desired_steering_angle, heading_error, cross_error, steering_angle_stanley
    
    def update_vehicle_state(self, desired_speed, desired_steering_angle):
        # 更新车辆位置和航向角
        self.vehicle_x += desired_speed * math.cos(self.vehicle_yaw) * self.dt
        self.vehicle_y += desired_speed * math.sin(self.vehicle_yaw) * self.dt
        self.vehicle_yaw += (desired_speed / self.wheelbase) * math.tan(desired_steering_angle) * self.dt
        
    def record_data(self, t, distance, desired_speed, desired_steering_angle):
        self.time_history.append(t)
        self.vehicle_x_history.append(self.vehicle_x)
        self.vehicle_y_history.append(self.vehicle_y)
        self.vehicle_yaw_history.append(self.vehicle_yaw)
        self.vehicle_v_history.append(desired_speed)
        self.vehicle_steering_history.append(desired_steering_angle)
        self.pedestrian_x_history.append(self.pedestrian_x)
        self.pedestrian_y_history.append(self.pedestrian_y)
        self.distance_history.append(distance)
        self.desired_speed_history.append(desired_speed)
        self.desired_steering_history.append(desired_steering_angle)
        
    def plot_results(self):
        # 创建一个包含七个子图的单列图形，并设置较小的图形尺寸
        fig, axs = plt.subplots(7, 1, figsize=(12, 35), constrained_layout=True)

        # 定义统一的字体大小
        label_fontsize = 10
        title_fontsize = 12
        legend_fontsize = 9

        # 第一个子图：地图（轨迹图）
        axs[0].plot(self.vehicle_x_history, self.vehicle_y_history, label='Vehicle Path')
        axs[0].plot(self.pedestrian_x_history, self.pedestrian_y_history, 'r', label='Pedestrian Path', alpha=0.5)
        axs[0].set_xlabel('X Position [m]', fontsize=label_fontsize)
        axs[0].set_ylabel('Y Position [m]', fontsize=label_fontsize)
        axs[0].set_title(f'Vehicle and Pedestrian Trajectories (Case {self.case_number})', fontsize=title_fontsize)
        axs[0].legend(fontsize=legend_fontsize)
        axs[0].grid(True)

        # 第二个子图：距离和跟踪距离
        axs[1].plot(self.time_history, self.distance_history, label='Distance to Pedestrian')
        axs[1].axhline(y=self.desired_distance, color='r', linestyle='--', label='Desired Distance')
        axs[1].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[1].set_ylabel('Distance [m]', fontsize=label_fontsize)
        axs[1].set_title('Distance to Pedestrian over Time', fontsize=title_fontsize)
        axs[1].legend(fontsize=legend_fontsize)
        axs[1].grid(True)

        # 第三个子图：速度
        axs[2].plot(self.time_history, self.vehicle_v_history, label='Vehicle Speed')
        axs[2].plot(self.time_history, self.pedestrian_v_history, label='Pedestrian Speed')
        axs[2].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[2].set_ylabel('Speed [m/s]', fontsize=label_fontsize)
        axs[2].set_title('Speed over Time', fontsize=title_fontsize)
        axs[2].legend(fontsize=legend_fontsize)
        axs[2].grid(True)

        # 第四个子图：加速度
        axs[3].plot(self.time_history, self.vehicle_acc_history, label='Vehicle Acceleration')
        axs[3].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[3].set_ylabel('Acceleration [m/s²]', fontsize=label_fontsize)
        axs[3].set_title('Vehicle Acceleration over Time', fontsize=title_fontsize)
        axs[3].legend(fontsize=legend_fontsize)
        axs[3].grid(True)

        # 第五个子图：转向角
        axs[4].plot(self.time_history, np.degrees(self.vehicle_steering_history), label='Steering Angle')
        axs[4].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[4].set_ylabel('Steering Angle [deg]', fontsize=label_fontsize)
        axs[4].set_title('Steering Angle over Time', fontsize=title_fontsize)
        axs[4].legend(fontsize=legend_fontsize)
        axs[4].grid(True)

        # 第六个子图：Kp 和 Kd 项的贡献
        axs[5].plot(self.time_history, self.proportional_history, label='Proportional Contribution (Kp*e)')
        axs[5].plot(self.time_history, self.derivative_history, label='Derivative Contribution (Kd*de/dt)')
        axs[5].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[5].set_ylabel('Contribution [m/s]', fontsize=label_fontsize)
        axs[5].set_title('Contributions of Kp and Kd Terms over Time', fontsize=title_fontsize)
        axs[5].legend(fontsize=legend_fontsize)
        axs[5].grid(True)

        # 第七个子图：停止距离、时间头距和所需距离
        axs[6].plot(self.time_history, self.d_stop, label='Stop Distance')
        axs[6].plot(self.time_history, self.d_headway, label='Time Headway Distance')
        axs[6].plot(self.time_history, self.d_required, label='Required Distance')
        axs[6].set_xlabel('Time [s]', fontsize=label_fontsize)
        axs[6].set_ylabel('Distance [m]', fontsize=label_fontsize)
        axs[6].set_title('Stop Distance, Time Headway Distance, and Required Distance over Time', fontsize=title_fontsize)
        axs[6].legend(fontsize=legend_fontsize)
        axs[6].grid(True)

        # 调整整体布局以增加子图之间的间距
        plt.subplots_adjust(hspace=0.4)

        plt.show()


if __name__ == '__main__':
    # 设置案例编号
    case_number = 1  # 选择案例 1 或 2
    simulator = VehicleSimulator(case_number=case_number)
    print(f"Running Case {case_number}...")
    simulator.run_simulation()
