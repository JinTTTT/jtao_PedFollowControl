import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import dlti, dstep

# 定义连续时间传递函数的系数
num = [1.0]  # 分子
den = [1.0, 1.0]  # 分母

# 设置采样时间
T_s = 0.1  # 采样时间，单位秒

# 离散化连续时间传递函数使用前向欧拉方法
# 连续传递函数 H(s) = num / den = 1 / (s + 1)
# 前向欧拉替换 s = (z - 1) / T_s
# H(z) = 1 / ((z - 1) / T_s + 1) = T_s / (z - 1 + T_s)

# 计算离散传递函数的系数
num_d = [T_s]  # 分子系数
den_d = [T_s + 1, -1]  # 分母系数

# 创建离散时间系统
system = dlti(num_d, den_d)

# 计算阶跃响应
t, y_tf = dstep(system)

# 模拟阶跃响应使用差分方程
steps = len(t)  # 模拟步数
y_diff = np.zeros(steps)
u = np.ones(steps)  # 单位阶跃输入

# 差分方程迭代
for k in range(1, steps):
    y_diff[k] = (T_s * u[k] + y_diff[k-1]) / (1 + T_s)

# 绘图对比
plt.figure()
plt.step(t, np.squeeze(y_tf), where='post', label='Transfer Function Response')
plt.step(t, y_diff, where='post', linestyle='--', label='Difference Equation Response')
plt.title('Step Response Comparison')
plt.xlabel('Time steps')
plt.ylabel('Response')
plt.legend()
plt.grid(True)
plt.show()
