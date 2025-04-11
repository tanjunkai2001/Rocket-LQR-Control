import numpy as np
# from scipy.linalg import solve_discrete_are  # 离散系统
from scipy.linalg import solve_continuous_are  # 连续系统

mass = 20.0  # 火箭质量(kg)
d = 0.55  # 发动机推力作用点到火箭质心距离(m)
r = 0.1  # 火箭半径(m) RCS滚转控制力矩长度
# 火箭转动惯量
Jxx = 2.88279167
Jyy = 2.88279167
Jzz = 0.13225
g = 9.81  # 重力加速度(m/s^2)
t_hov = mass * g  # 悬停时发动机推力大小
# 系统连续状态矩阵A 12x12
A = np.zeros((12, 12))
A[0:3, 3:6] = np.eye(3)
A[3:6, 6:9] = np.array([[0.0, g, 0.0], [-g, 0.0, 0.0], [0.0, 0.0, 0.0]])
A[6:9, 9:12] = np.eye(3)
# 系统控制矩阵B 12x4
B = np.zeros((12, 4))
B[3:6, 0:4] = np.array([[0.0, -t_hov/mass, 0.0, 0.0], [t_hov/mass, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0/mass]])
B[9:12, 0:4] = np.array([[t_hov/(Jxx*d), 0.0, 0.0, 0.0], [0.0, t_hov/(Jyy*d), 0.0, 0.0], [0.0, 0.0, 1.0/(Jzz), 0.0]])
# 状态权重矩阵Q 12x12
q_position = 1.0
q_body_velocity = 1.0
q_euler = 1.0
q_angular_velocity = 1.0
Q = np.diag([q_position, q_position, q_position, q_body_velocity, q_body_velocity, q_body_velocity, q_euler, q_euler, q_euler, q_angular_velocity, q_angular_velocity, q_angular_velocity])
# 控制权重矩阵R 4x4
r_alpha = 1.0
r_beta = 1.0
r_gamma = 1.0
r_thrust = 1.0
R = np.diag([r_alpha, r_beta, r_gamma, r_thrust])
# 使用连续系统进行计算
S = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ (B.T @ S)

# 离散化
# P = solve_discrete_are(A, B, Q, R)
# K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)

# print(K)

# [[-7.19480065e-15 -1.00000000e+00  3.14406041e-15 -6.47567190e-15
#   -1.47773134e+00  1.34052896e-14  5.80599907e+00 -4.61094565e-15
#   -6.32328963e-16  1.16540671e+00 -4.34476206e-16  2.19752951e-15]
#  [ 1.00000000e+00 -3.65238954e-15 -3.95321689e-16  1.47773134e+00
#   -7.15145914e-15 -6.64804736e-15  5.44926652e-15  5.80599907e+00
#    2.70068857e-16  6.45838740e-16  1.16540671e+00 -4.09241522e-16]
#  [-1.14595249e-15 -6.65221429e-16  2.98280271e-15 -1.13649669e-16
#   -4.62591501e-16  2.48412657e-15  1.10976359e-15 -9.95745730e-16
#    1.00000000e+00  1.70954198e-16 -3.40167420e-17  1.12449989e+00]
#  [-2.78473776e-17 -9.17591005e-17  1.00000000e+00 -5.06503499e-17
#    3.18932543e-17  6.40312424e+00 -4.17651132e-16 -2.51774752e-16
#   -3.70267635e-16  2.88816067e-18 -6.70161294e-18  1.64262869e-17]]