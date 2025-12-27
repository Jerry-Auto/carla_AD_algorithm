
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取数据
data = pd.read_csv('lateral_control_simulation.csv')

# 创建图形
plt.figure(figsize=(15, 10))

# 1. 轨迹跟踪图
plt.subplot(3, 2, 1)
plt.plot(data['X'], data['Y'], 'b-', linewidth=2, label='Vehicle Path')
# 生成参考轨迹用于显示
x_ref = np.linspace(data['X'].min(), data['X'].max(), 100)
y_ref = 2.0 * np.sin(0.2 * x_ref)
plt.plot(x_ref, y_ref, 'r--', linewidth=1.5, label='Target Path')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Tracking')
plt.legend()
plt.grid(True)
plt.axis('equal')

# 2. 横向误差
plt.subplot(3, 2, 2)
plt.plot(data['Time'], data['LateralError'], 'g-', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Lateral Error (m)')
plt.title('Lateral Error vs Time')
plt.grid(True)

# 3. 方向盘转角
plt.subplot(3, 2, 3)
steer_deg = data['SteerAngle'] * 180 / np.pi
plt.plot(data['Time'], steer_deg, 'm-', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Steer Angle (deg)')
plt.title('Steering Command')
plt.grid(True)

# 4. 航向角
plt.subplot(3, 2, 4)
heading_deg = data['Heading'] * 180 / np.pi
target_heading_deg = data['TargetHeading'] * 180 / np.pi
plt.plot(data['Time'], heading_deg, 'b-', linewidth=2, label='Vehicle')
plt.plot(data['Time'], target_heading_deg, 'r--', linewidth=1.5, label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading (deg)')
plt.title('Heading Comparison')
plt.legend()
plt.grid(True)

# 5. 速度
plt.subplot(3, 2, 5)
plt.plot(data['Time'], data['Velocity'], 'c-', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Vehicle Speed')
plt.grid(True)

# 6. 误差直方图
plt.subplot(3, 2, 6)
plt.hist(data['LateralError'], bins=20, alpha=0.7, color='orange', edgecolor='black')
plt.xlabel('Lateral Error (m)')
plt.ylabel('Frequency')
plt.title('Lateral Error Distribution')
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('lateral_control_results.png', dpi=300, bbox_inches='tight')
plt.show()

print("=== 性能统计 ===")
print(f"最大横向误差: {data['LateralError'].abs().max():.3f} m")
print(f"平均横向误差: {data['LateralError'].abs().mean():.3f} m")
print(f"横向误差标准差: {data['LateralError'].std():.3f} m")
print(f"最大方向盘转角: {steer_deg.abs().max():.1f} °")
print(f"平均方向盘转角: {steer_deg.abs().mean():.1f} °")

# 计算收敛时间（误差小于0.1m的时间）
if (data['LateralError'].abs() < 0.1).any():
    convergence_time = data.loc[data['LateralError'].abs() < 0.1, 'Time'].min()
    print(f"收敛到0.1m内的时间: {convergence_time:.1f} s")
else:
    print("未收敛到0.1m内")
