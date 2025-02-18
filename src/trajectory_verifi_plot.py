"""
@brief：绘制机械臂轨迹数据的3D图
"""
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 读取CSV数据
df = pd.read_csv(r'/home/le/dev_ws/src/rokae_move/single_trajectory_data.csv')

# 创建3D图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 将DataFrame数据转换为numpy数组
x = df['x'].to_numpy()
y = df['y'].to_numpy()
z = df['z'].to_numpy()

# 绘制轨迹
ax.plot(x, y, z, linewidth=2)

# 设置轴标签
ax.set_xlabel('X（m）')
ax.set_ylabel('Y（m）')
ax.set_zlabel('Z（m）')
ax.set_title('Robot Arm Trajectory')

plt.show()