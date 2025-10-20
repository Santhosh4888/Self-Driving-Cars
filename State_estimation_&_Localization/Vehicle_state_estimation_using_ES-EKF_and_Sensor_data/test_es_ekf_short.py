import pickle
import numpy as np
from rotations import Quaternion
from es_ekf import measurement_update

# Load data
with open('data/pt1_data.pkl', 'rb') as file:
    data = pickle.load(file)
gt = data['gt']
imu_f = data['imu_f']
imu_w = data['imu_w']
gnss = data['gnss']
lidar = data['lidar']

# Set up initial state
p_est = np.zeros([imu_f.data.shape[0], 3])
v_est = np.zeros([imu_f.data.shape[0], 3])
q_est = np.zeros([imu_f.data.shape[0], 4])
p_cov = np.zeros([imu_f.data.shape[0], 9, 9])
p_est[0] = gt.p[0]
v_est[0] = gt.v[0]
q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
p_cov[0] = np.zeros(9)
gnss_i = 0
lidar_i = 0

# Minimal ES-EKF loop for 5 steps
var_imu_f = 1.0
var_imu_w = 1.25
var_gnss  = 0.1
var_lidar = 50.0
g = np.array([0, 0, -9.81])
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)

for k in range(1, 6):
    delta_t = imu_f.t[k] - imu_f.t[k - 1]
    C_ns = Quaternion(*q_est[k-1]).to_mat()
    C_ns_d_f_km = np.dot(C_ns, imu_f.data[k-1])
    p_est[k] = p_est[k-1] + delta_t * v_est[k-1] + (delta_t**2) / 2 * (C_ns_d_f_km + g)
    v_est[k] = v_est[k-1] + delta_t * (C_ns_d_f_km + g)
    q_fr_w = Quaternion(axis_angle=imu_w.data[k-1] * delta_t)
    q_est[k] = q_fr_w.quat_mult_right(q_est[k-1])
    f_ja_km = np.identity(9)
    f_ja_km[0:3, 3:6] = np.identity(3) * delta_t
    f_ja_km[3:6, 6:9] = -np.zeros((3,3)) * delta_t
    q_cov_km = np.identity(6)
    q_cov_km[0:3,0:3] *=  delta_t**2 * np.eye(3) * var_imu_f
    q_cov_km[3:6, 3:6] *= delta_t**2 * np.eye(3) * var_imu_w
    p_cov[k] = f_ja_km.dot(p_cov[k-1]).dot(f_ja_km.T) + l_jac.dot(q_cov_km).dot(l_jac.T)
print('Minimal ES-EKF loop ran successfully for 5 steps.')
