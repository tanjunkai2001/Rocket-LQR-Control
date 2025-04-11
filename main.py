import mujoco
import mujoco.viewer as viewer
import numpy as np
import quaternion
from scipy.linalg import solve_continuous_are  # 连续系统

MASS = 20.0 # 火箭质量 20kg(假设质量不变)
MAX_THRUST = 400.0 # 最大推力 400N
MIN_THRUST = 0.0 # 最小推力 0N
MAX_ANGLE_DEG = 30.0 # 单轴矢量最大摆动角度 30度
MAX_ANGLE_RAD = np.deg2rad(MAX_ANGLE_DEG) # 单轴矢量最大摆动角度

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
q_position_xy = 0.2
q_position_z = 1000.0
q_body_velocity_xy = 0.0001
q_body_velocity_z = 10.0
q_euler = 2.0
q_angular_velocity = 0.5
Q = np.diag([q_position_xy, q_position_xy, q_position_z, q_body_velocity_xy, q_body_velocity_xy, q_body_velocity_z, q_euler, q_euler, q_euler, q_angular_velocity, q_angular_velocity, q_angular_velocity])
# 控制权重矩阵R 4x4
r_alpha = 0.5
r_beta = 0.5
r_gamma = 2.0
r_thrust = 1.0
R = np.diag([r_alpha, r_beta, r_gamma, r_thrust])
# 使用连续系统进行计算
S = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ (B.T @ S)

# 设置矢量推力
# alpha: 推力矢量第一个旋转轴角度(rad) 正值意味着产生绕X轴正向旋转力矩
# beta: 推力矢量第二个旋转轴角度(rad) 正值意味着产生绕Y轴正向旋转力矩
# thrust: 推力大小(N)
def set_engine_thrust(data, alpha, beta, gamma, thrust):
	# 限制推力大小
	if thrust > MAX_THRUST:
		thrust = MAX_THRUST
	if thrust < MIN_THRUST:
		thrust = MIN_THRUST
	# 限制摆动角度
	if alpha > MAX_ANGLE_RAD:
		alpha = MAX_ANGLE_RAD
	if alpha < -MAX_ANGLE_RAD:
		alpha = -MAX_ANGLE_RAD
	if beta > MAX_ANGLE_RAD:
		beta = MAX_ANGLE_RAD
	if beta < -MAX_ANGLE_RAD:
		beta = -MAX_ANGLE_RAD
	# 计算推力矢量 使用二自由度矢量摆动
	thrust_x = - thrust * np.cos(alpha) * np.sin(beta)
	thrust_y = thrust * np.sin(alpha)
	thrust_z = thrust * np.cos(alpha) * np.cos(beta)
	# 设置推力
	data.actuator('thrust_x').ctrl[0] = thrust_x
	data.actuator('thrust_y').ctrl[0] = thrust_y
	data.actuator('thrust_z').ctrl[0] = thrust_z
	data.actuator('RCS').ctrl[0] = gamma

goal_pos = np.array([-1.0, 0.0, 0.3])
engine_on = False
height = 0.0
height_vel = 0.0
has_landed = False
control_mode = 'takeoff and land'
# control_mode = 'position control'

def rocket_control_callback(m, d):
	global K, goal_pos, engine_on, height, height_vel, has_landed, control_mode
	_time = d.time
	# for reset
	if _time < 1.0:
		has_landed = False
	if control_mode == 'position control':
		goal_pos = np.array([0.0, 0.0, 2.0])
		engine_on = True
	elif control_mode == 'takeoff and land':
		if _time < 3.0:
			engine_on = False
		elif _time < 7.0:
			engine_on = True
			goal_pos = np.array([0.0, 0.0, 2.0])
		elif _time < 13.0:
			if has_landed == False:
				engine_on = True
				goal_pos = np.array([1.0, 0.0, 0.3])
				if height < 0.01 and abs(height_vel) < 0.01:  # Land
					engine_on = False
					has_landed = True

	# 全状态获取
	# 世界坐标系位置 qpos[0:3]
	x_pos = d.qpos[0] - goal_pos[0]
	y_pos = d.qpos[1] - goal_pos[1]
	z_pos = d.qpos[2] - goal_pos[2]
	world_pos = np.array([x_pos, y_pos, z_pos])
	height = z_pos
	# 四元数 qpos[3:7]
	quat_w = d.qpos[3]	
	quat_x = d.qpos[4]
	quat_y = d.qpos[5]
	quat_z = d.qpos[6]
	# 世界坐标系速度 qvel[0:3]
	x_vel = d.qvel[0]
	y_vel = d.qvel[1]
	z_vel = d.qvel[2]
	world_vel = np.array([x_vel, y_vel, z_vel])
	height_vel = z_vel
	# 机体坐标系角速度 qvel[3:6]
	omega_x = d.qvel[3]
	omega_y = d.qvel[4]
	omega_z = d.qvel[5]
	omega = np.array([omega_x, omega_y, omega_z])
	# 计算旋转矩阵(机体坐标系转换为世界坐标系)
	_quat = quaternion.quaternion(quat_w, quat_x, quat_y, quat_z)
	Rwb = quaternion.as_rotation_matrix(_quat)
	Rbw = Rwb.T
	# 计算欧拉角(使用旋转矩阵转换)
	_rotmat = Rwb
	roll = np.arctan2(_rotmat[2][1], _rotmat[2][2])
	pitch = -np.arcsin(_rotmat[2][0])
	yaw = np.arctan2(_rotmat[1][0], _rotmat[0][0])
	euler = np.array([roll, pitch, yaw])
	# print(f"Euler: roll:{roll:.2f} pitch:{pitch:.2f} yaw:{yaw:.2f}")
	# 计算机体坐标系速度
	body_vel = np.dot(Rbw, world_vel)
	# 构建12维度状态向量
	state = np.concatenate((world_pos, body_vel, euler, omega))
	# 计算控制量
	u = -np.dot(K, state)
	# 控制量施加常规悬停推力
	u[3] += t_hov

	if engine_on == True:	
	# 施加控制量
		set_engine_thrust(d, u[0], u[1], u[2], u[3])
	else:
		set_engine_thrust(d, 0, 0, 0, 0)

# 加载模型函数
def load_callback(m=None, d=None):
	mujoco.set_mjcb_control(None)  # 清除控制回调函数
	m = mujoco.MjModel.from_xml_path('vtol/scene.xml')
	d = mujoco.MjData(m)
	print("Set Control Callback")
	if m is not None:
		mujoco.set_mjcb_control(lambda m, d: rocket_control_callback(m, d))
	return m, d

if __name__ == '__main__':
  	viewer.launch(loader=load_callback)
