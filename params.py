import numpy as np

robot_params = {}

robot_params['tractor'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.,    100.,  100.]),
                        'kd':  np.array([10.,    10.,    10.,   10.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': -0.,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 5000}

robot_params['tractor0'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.,    100.,  100.]),
                        'kd':  np.array([10.,    10.,    10.,   10.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': -0.,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 5000}

robot_params['tractor1'] ={'dt': 0.01,
                        'kp': np.array([100.,   100.,    100.,  100.]),
                        'kd':  np.array([10.,    10.,    10.,   10.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'], # caster wheels are passive joints
                        'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                        'spawn_x': 5.,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 5000}

verbose = False
plotting = True