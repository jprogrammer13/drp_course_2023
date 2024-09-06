import numpy as np

robot_params = {}

robot_params['tractor0'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': -0.,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 30000}

robot_params['tractor1'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 1.,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

robot_params['tractor2'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': -1,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}


robot_params['tractor3'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 1,
                            'spawn_y': 0.05,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

robot_params['tractor4'] = {'dt': 0.01,
                            'kp': np.array([100.,   100.]),
                            'kd':  np.array([10.,    10.]),
                            'q_0':  np.array([0, 0]),
                            # caster wheels are passive joints
                            'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint'],
                            'ee_frames': ['front_left_wheel', 'front_right_wheel'],
                            'spawn_x': 0,
                            'spawn_y': -1,
                            'spawn_z': 0.25,
                            'spawn_yaw': 0.1,
                            'buffer_size': 100}

exp_params = {0.09041: {"C1": -1.2255, "C2": -8.1813},
              0.1: {"C1": -1.1280, "C2": -8.2517},
              0.13349: {"C1": -0.8796, "C2": -8.4115},
              0.1568: {"C1": -0.7650, "C2": -8.4872}}

global_dt = 0.01

verbose = False
plotting = True
