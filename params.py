import numpy as np

robot_params = {}

robot_params['tractor0'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.]),
                        'kd':  np.array([10.,    10.]),
                        'q_0':  np.array([0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': -0.,
                        'spawn_y': 0.05,
                        'spawn_z': 0.25,
                        'spawn_yaw': 0.1,
                        'buffer_size': 30000}

robot_params['tractor1'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.]),
                        'kd':  np.array([10.,    10.]),
                        'q_0':  np.array([0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': 1.,
                        'spawn_y': 0.05,
                        'spawn_z': 0.25,
                        'spawn_yaw': 0.1,
                        'buffer_size': 100}

robot_params['tractor2'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.]),
                        'kd':  np.array([10.,    10.]),
                        'q_0':  np.array([0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': -1,
                        'spawn_y': 0.05,
                        'spawn_z': 0.25,
                        'spawn_yaw': 0.1,
                        'buffer_size': 100}


robot_params['tractor3'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.]),
                        'kd':  np.array([10.,    10.]),
                        'q_0':  np.array([0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': 1,
                        'spawn_y': 0.05,
                        'spawn_z': 0.25,
                        'spawn_yaw': 0.1,
                        'buffer_size': 100}

robot_params['tractor4'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.]),
                        'kd':  np.array([10.,    10.]),
                        'q_0':  np.array([0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': 0,
                        'spawn_y': -1,
                        'spawn_z': 0.25,
                        'spawn_yaw': 0.1,
                        'buffer_size': 100}

global_dt = 0.01

verbose = False
plotting = True