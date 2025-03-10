import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# 定义连续时间系统参数
a = 0.3969
b = 0.2394
num_c = [1]  # 分子系数
den_c = [a, b, 1]  # 分母系数，as^2 + bs + 1 的形式

# 创建连续时间传递函数
G_s = ctl.TransferFunction(num_c, den_c)

# 计算阶跃响应
t_c, y_c = ctl.step_response(G_s)

# 绘制阶跃响应
plt.figure()
plt.plot(t_c, y_c, label='Continuous-Time Step Response', linewidth=2)
plt.title('Continuous-Time Step Response')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()
plt.show()