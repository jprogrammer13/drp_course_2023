# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
from base_controllers.utils.common_functions import checkRosMaster
# from base_controllers.utils.common_functions import getRobotModelFloating
from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator import TrackedVehicleSimulator, Ground
from optim_interfaces.srv import Optim, OptimRequest
from base_controllers.components.coppelia_manager import CoppeliaManager
import pinocchio as pin
import catboost as cb
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from termcolor import colored
from base_controllers.tracked_robot.velocity_generator import VelocityGenerator
from base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
from base_controllers.tracked_robot.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from base_controllers.tracked_robot.utils import constants as constants
from base_controllers.utils.math_tools import unwrap_angle
from base_controllers.utils.custom_robot_wrapper import RobotWrapper

from matplotlib import pyplot as plt
from numpy import nan
# gazebo messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
# gazebo services
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest


from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import ApplyBodyWrench
from std_srvs.srv import Empty
import rospkg
import rosgraph

import sys
import os
import params as conf
from base_controllers.utils.common_functions import plotFrameLinear, plotJoint, sendStaticTransform, launchFileGeneric, launchFileNode
from base_controllers.base_controller import BaseController
from base_controllers.utils.ros_publish import RosPub

import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=1000, suppress=True)


robotName = "tractor"  # needs to inherit BaseController


class GenericSimulator(BaseController):

    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf=conf)
        self.torque_control = False

        # 'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.ControlType = 'OPEN_LOOP'
        # Parameters for open loop identification
        self.IDENT_TYPE = 'NONE'  # 'V_OMEGA', 'NONE'
        self.IDENT_DIRECTION = 'left'  # used only when OPEN_LOOP
        self.IDENT_LONG_SPEED = 0.1  # 0.05:0.05:0.4
        self.IDENT_WHEEL_L = 4.5  # -4.5:0.5:4.5

        # initial pose
        self.p0 = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                           conf.robot_params[self.robot_name]['spawn_y'],
                           conf.robot_params[self.robot_name]['spawn_z']])

        # target for matlab trajectory generation (dubins/optimization)
        self.pf = np.array([2., 2.5, 0.])

        self.GRAVITY_COMPENSATION = False
        self.SAVE_BAGS = False
        self.LONG_SLIP_COMPENSATION = 'NONE'  # 'NN', 'EXP', 'NONE'
        self.NAVIGATION = False
        self.USE_GUI = True  # false does not work in headless mode

    def initVars(self):
        super().initVars()

        # regressor
        self.model = cb.CatBoostRegressor()
        # laod model
        try:
            self.model_beta_l.load_model(
                os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/controllers/regressor/model_beta_l.cb')
            self.model_beta_r.load_model(
                os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/controllers/regressor/model_beta_r.cb')
        except:
            print(colored(
                "need to generate the models with running tracked_robot/controller/regressor/model_slippage_updated.py"))
        # add your variables to initialize here
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V = 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(self.robot.na)
        self.ctrl_v_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.ctrl_omega_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.v_d_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.omega_d_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.V_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.V_dot_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.des_x = 0.
        self.des_y = 0.
        self.des_theta = 0.
        self.beta_l = 0.
        self.beta_r = 0.
        self.alpha = 0.
        self.alpha_control = 0.
        self.radius = 0.
        self.beta_l_control = 0.
        self.beta_r_control = 0.

        self.state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full(
            (3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.beta_l_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_control_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.radius_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_l_control_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_control_log = np.empty(
            (conf.robot_params[self.robot_name]['buffer_size'])) * nan

    def reset_joints(self, q0, joint_names=None):
        # create the message
        req_reset_joints = SetModelConfigurationRequest()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        if joint_names == None:
            req_reset_joints.joint_names = self.joint_names
        else:
            req_reset_joints.joint_names = joint_names
        req_reset_joints.joint_positions = q0
        self.reset_joints_client(req_reset_joints)
        print(colored(f"---------Resetting Joints to: "+str(q0), "blue"))

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            # add your logs here
            self.ctrl_v_log[self.log_counter] = self.ctrl_v
            self.ctrl_omega_log[self.log_counter] = self.ctrl_omega
            self.v_d_log[self.log_counter] = self.v_d
            self.omega_d_log[self.log_counter] = self.omega_d
            self.V_log[self.log_counter] = self.V
            self.V_dot_log[self.log_counter] = self.V_dot
            self.des_state_log[0, self.log_counter] = self.des_x
            self.des_state_log[1, self.log_counter] = self.des_y
            self.des_state_log[2, self.log_counter] = self.des_theta
            self.state_log[0,
                           self.log_counter] = self.basePoseW[self.u.sp_crd["LX"]]
            self.state_log[1,
                           self.log_counter] = self.basePoseW[self.u.sp_crd["LY"]]
            self.state_log[2,
                           self.log_counter] = self.basePoseW[self.u.sp_crd["AZ"]]

            self.alpha_log[self.log_counter] = self.alpha
            self.beta_l_log[self.log_counter] = self.beta_l
            self.beta_r_log[self.log_counter] = self.beta_r

            self.alpha_control_log[self.log_counter] = self.alpha_control
            self.beta_l_control_log[self.log_counter] = self.beta_l_control
            self.beta_r_control_log[self.log_counter] = self.beta_r_control
            self.radius_log[self.log_counter] = self.radius
        super().logData()

    def getRobotModelFloating(self, robot_name="hyq"):
            ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n'
            path = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
            if rosgraph.is_master_online():
                try:
                    urdf = ros.get_param('/robot_description')
                    print("URDF generated_commons")
                    os.makedirs(path + "/robot_urdf/generated_urdf/", exist_ok=True)
                    urdf_location = path + "/robot_urdf/generated_urdf/" + robot_name+ ".urdf"
                    print(urdf_location)
                    text_file = open(urdf_location, "w")
                    text_file.write(urdf)
                    text_file.close()
                    robot = RobotWrapper.BuildFromURDF(urdf_location, root_joint=pin.JointModelFreeFlyer())
                except:
                    print('Issues in URDF generation for Pinocchio, did not succeed')

            return robot

    def startSimulator(self):
        # os.system("killall rosmaster rviz gzserver coppeliaSim")
        # launch roscore
        checkRosMaster()
        ros.sleep(1.5)
        # run robot state publisher + load robot description + rviz
        groundParams = Ground()
        self.tracked_vehicle_simulator = TrackedVehicleSimulator(
            dt=conf.robot_params[p.robot_name]['dt'], ground=groundParams)
        self.tracked_vehicle_simulator.initSimulation(vbody_init=np.array(
            [0, 0, 0.0]), pose_init=self.p0)  # TODO make this a parameter
        self.robot = self.getRobotModelFloating(self.robot_name)
        # instantiating additional publishers
        self.joint_pub = ros.Publisher(
            "/" + self.robot_name + "/joint_states", JointState, queue_size=1)
        self.groundtruth_pub = ros.Publisher(
            "/" + self.robot_name + "/ground_truth", Odometry, queue_size=1, tcp_nodelay=True)

    def loadModelAndPublishers(self, xacro_path=None):
        
        self.robot = self.getRobotModelFloating(self.robot_name)
        if xacro_path is None:
            xacro_path = rospkg.RosPack().get_path(
                self.robot_name[:-1] + '_description') + '/robots/' + self.robot_name + '.urdf.xacro'
        else:
            print("loading custom xacro path: ", xacro_path)
        self.robot = self.getRobotModelFloating(self.robot_name)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)

        self.pub_des_jstate = ros.Publisher(
            "/command", JointState, queue_size=1, tcp_nodelay=True)
        # freeze base  and pause simulation service
        self.reset_world = ros.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy(
            '/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy(
            '/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy(
            '/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy(
            '/gazebo/unpause_physics', Empty)
        self.u.putIntoGlobalParamServer("verbose", self.verbose)

        self.apply_body_wrench = ros.ServiceProxy(
            '/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        self.reset_joints_client = ros.ServiceProxy(
            '/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel = ros.Publisher(
            "/des_vel", JointState, queue_size=1, tcp_nodelay=True)

        if self.SAVE_BAGS:
            if p.ControlType == 'OPEN_LOOP':
                if p.IDENT_TYPE == 'V_OMEGA':
                    bag_name = f"ident_sim_longv_{p.IDENT_LONG_SPEED}_{p.IDENT_DIRECTION}.bag"
                else:
                    bag_name = f"ident_sim_wheelL_{p.IDENT_WHEEL_L}.bag"
            else:
                bag_name = f"{p.ControlType}_Long_{self.LONG_SLIP_COMPENSATION}.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)

    # This will be used instead of the basecontroller one, I do it just to check frequency

    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

        self.check_time = ros.Time.now().to_sec()

    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        os.system("killall rosmaster rviz")
        super().deregister_node()

    def startupProcedure(self):
        # Biral
        # fixed height TODO change this when on slopes
        self.basePoseW[2] = 0.25
        self.broadcast_world = False
        self.slow_down_factor = 1
        # loop frequency
        self.rate = ros.Rate(
            1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))
        pass

    def plotData(self):
        if conf.plotting:
            # xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :],
                     p.des_state_log[1, :], "-r", label="desired")
            plt.plot(p.state_log[0, :], p.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            # # command plot
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.ctrl_v_log, "-b", label="REAL")
            plt.plot(p.time_log, p.v_d_log, "-r", label="desired")
            plt.legend()
            plt.ylabel("linear velocity[m/s]")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.ctrl_omega_log, "-b", label="REAL")
            plt.plot(p.time_log, p.omega_d_log, "-r", label="desired")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("angular velocity[rad/s]")
            plt.grid(True)

            # plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            # joint velocities with limits
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.qd_log[0, :], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[0, :], "-r",  linewidth=4)
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_L")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.qd_log[1, :], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[1, :], "-r",  linewidth=4)
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_R")
            plt.grid(True)

            # states plot
            plotFrameLinear(name='position', time_log=p.time_log,
                            des_Pose_log=p.des_state_log, Pose_log=p.state_log)
            plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=np.vstack(
                (p.baseTwistW_log[:2, :], p.baseTwistW_log[5, :])))

            # slippage vars
            plt.figure()
            plt.subplot(4, 1, 1)
            plt.plot(self.time_log, self.beta_l_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_l_control_log,
                     "-r", label="control")
            plt.ylabel("beta_l")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 2)
            plt.plot(self.time_log, self.beta_r_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_r_control_log,
                     "-r", label="control")
            plt.ylabel("beta_r")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 3)
            plt.plot(self.time_log, self.alpha_log, "-b", label="real")
            plt.plot(self.time_log, self.alpha_control_log,
                     "-r", label="control")
            plt.ylabel("alpha")
            plt.ylim([-1, 1])
            plt.grid(True)
            plt.legend()
            plt.subplot(4, 1, 4)
            plt.plot(self.time_log, self.radius_log, "-b")
            plt.ylim([-1, 1])
            plt.ylabel("radius")
            plt.grid(True)

            if p.ControlType == 'CLOSED_LOOP':
                # tracking errors
                self.log_e_x, self.log_e_y, self.log_e_theta = self.controller.getErrors()
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(np.sqrt(np.power(self.log_e_x, 2) +
                         np.power(self.log_e_y, 2)), "-b")
                plt.ylabel("exy")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(self.log_e_theta, "-b")
                plt.ylabel("eth")
                plt.grid(True)

                #
                # # liapunov V
                # plt.figure()
                # plt.plot(p.time_log, p.V_log, "-b", label="REAL")
                # plt.legend()
                # plt.xlabel("time[sec]")
                # plt.ylabel("V liapunov")
                # # plt.axis("equal")
                # plt.grid(True)
                #
                # #base position
                # plotFrame('position', time_log=p.time_log, Pose_log=p.basePoseW_log,
                #           title='Base', frame='W', sharex=True)

    def mapToWheels(self, v_des, omega_des):
        #
        # # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)
        qd_des = np.zeros(2)
        qd_des[0] = (v_des - omega_des * constants.TRACK_WIDTH /
                     2)/constants.SPROCKET_RADIUS  # left front
        qd_des[1] = (v_des + omega_des * constants.TRACK_WIDTH /
                     2)/constants.SPROCKET_RADIUS  # right front

        # publish des commands as well
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.velocity = np.array([v_des, omega_des])
        self.des_vel.publish(msg)

        return qd_des
    # unwrap the joints states

    def unwrap(self):
        for i in range(self.robot.na):
            self.q[i], self.q_old[i] = unwrap_angle(self.q[i], self.q_old[i])

    def generateWheelTraj(self, wheel_l=-4.5):
        ####################################
        # OPEN LOOP wl , wr (from -4.5 to 4.5)
        ####################################
        wheel_l_vec = []
        wheel_r_vec = []
        change_interval = 3.
        nsamples = 12
        if wheel_l <= 0.:  # this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-constants.MAXSPEED_RADS_PULLEY,
                                  constants.MAXSPEED_RADS_PULLEY, nsamples)
        else:
            wheel_r = np.linspace(
                constants.MAXSPEED_RADS_PULLEY, -constants.MAXSPEED_RADS_PULLEY, nsamples)
        time = 0
        i = 0
        while True:
            time = np.round(time + conf.robot_params[p.robot_name]['dt'], 3)
            wheel_l_vec.append(wheel_l)
            wheel_r_vec.append(wheel_r[i])
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(wheel_r):
                break

        wheel_l_vec.append(0.0)
        wheel_r_vec.append(0.0)
        return wheel_l_vec, wheel_r_vec

    def generateOpenLoopTraj(self, R_initial=0.05, R_final=0.6, increment=0.025, dt=0.005, long_v=0.1, direction="left"):
        # only around 0.3
        change_interval = 6.
        increment = increment
        turning_radius_vec = np.arange(R_initial, R_final, increment)
        if direction == 'left':
            ang_w = np.round(long_v / turning_radius_vec, 3)  # [rad/s]
        else:
            ang_w = -np.round(long_v / turning_radius_vec, 3)  # [rad/s]
        omega_vec = []
        v_vec = []
        time = 0
        i = 0
        while True:
            time = np.round(time + dt, 3)
            omega_vec.append(ang_w[i])
            v_vec.append(long_v)
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(turning_radius_vec):
                break
        v_vec.append(0.0)
        omega_vec.append(0.0)
        return v_vec, omega_vec

    def estimateSlippages(self, W_baseTwist, theta, qd):

        wheel_L = qd[0]
        wheel_R = qd[1]
        w_vel_xy = np.zeros(2)
        w_vel_xy[0] = W_baseTwist[self.u.sp_crd["LX"]]
        w_vel_xy[1] = W_baseTwist[self.u.sp_crd["LY"]]
        omega = W_baseTwist[self.u.sp_crd["AZ"]]

        # compute BF velocity
        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
        b_vel_xy = (w_R_b.T).dot(w_vel_xy)
        b_vel_x = b_vel_xy[0]

        # track velocity  from encoder
        v_enc_l = constants.SPROCKET_RADIUS * wheel_L
        v_enc_r = constants.SPROCKET_RADIUS * wheel_R
        B = constants.TRACK_WIDTH

        v_track_l = b_vel_x - omega * B / 2
        v_track_r = b_vel_x + omega * B / 2

        # discrepancy bw what it turn out to be (real track) and what it
        # should be (desired) from encoder
        beta_l = v_enc_l-v_track_l
        beta_r = v_enc_r-v_track_r
        if (abs(b_vel_xy[1]) < 0.001) or (abs(b_vel_xy[0]) < 0.001):
            side_slip = 0.
        else:
            side_slip = math.atan2(b_vel_xy[1], b_vel_xy[0])

        return beta_l, beta_r, side_slip

    def computeGravityCompensation(self, roll, pitch):
        W = np.array([0., 0., constants.mass*9.81, 0., 0., 0.])
        track_discretization = 4
        grasp_matrix_left = np.zeros((6, 3*track_discretization))
        grasp_matrix_right = np.zeros((6, 3*track_discretization))
        b_pos_track_left_x = np.linspace(
            constants.b_left_track_start[0], constants.b_left_track_end[0], track_discretization)
        b_pos_track_right_x = np.linspace(
            constants.b_right_track_start[0], constants.b_right_track_end[0], track_discretization)
        # mapping to horizontal frame
        w_R_b = self.math_utils.eul2Rot(
            np.array([roll, pitch, self.basePoseW[self.u.sp_crd["AZ"]]]))

        # fill in grasp matrix
        for i in range(track_discretization):
            b_pos_track_left_i = np.array(
                [b_pos_track_left_x[i], constants.b_left_track_start[1], constants.b_left_track_start[2]])
            grasp_matrix_left[:3, i*3:i*3+3] = np.eye(3)
            grasp_matrix_left[3:, i * 3:i * 3 +
                              3] = pin.skew(w_R_b.dot(b_pos_track_left_i))
            # debug
            # self.ros_pub.add_arrow(p.basePoseW[:3], w_R_b.dot(b_pos_track_left_i), "red")

        for i in range(track_discretization):
            b_pos_track_right_i = np.array(
                [b_pos_track_right_x[i], constants.b_right_track_start[1], constants.b_right_track_start[2]])
            grasp_matrix_right[:3, i*3:i*3+3] = np.eye(3)
            grasp_matrix_right[3:, i * 3:i * 3 +
                               3] = pin.skew(w_R_b.dot(b_pos_track_right_i))

        # get track element forces distributiong load
        fi = np.linalg.pinv(
            np.hstack((grasp_matrix_left, grasp_matrix_right))).dot(W)

        # get contact frame
        ty = w_R_b.dot(np.array([0, 1., 0.]))
        n = w_R_b.dot(np.array([0., 0., 1.]))
        # projection on tx
        # fill in projection/sum matrix
        proj_matrix = np.zeros((3, 3*track_discretization))
        sum_matrix = np.zeros((3, 3*track_discretization))
        for i in range(track_discretization):
            # compute projections by outer product I-n*nt, project first on n-x plane then on x
            proj_matrix[:3, i*3:i*3+3] = (np.eye(3) - np.multiply.outer(n.ravel(), n.ravel())).dot(
                (np.eye(3) - np.multiply.outer(ty.ravel(), ty.ravel())))
            sum_matrix[:3, i*3:i*3+3] = np.eye(3)

        F_l = sum_matrix.dot(fi[:3*track_discretization])
        F_r = sum_matrix.dot(fi[3*track_discretization:])
        F_lx = proj_matrix.dot(fi[:3*track_discretization])
        F_rx = proj_matrix.dot(fi[3*track_discretization:])

        tau_g = np.array([F_lx*constants.SPROCKET_RADIUS,
                         F_rx*constants.SPROCKET_RADIUS])
        return tau_g, F_l, F_r

    def computeLongSlipCompensation(self, v, omega, qd_des, constants):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)

        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS*qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS*qd_des[1]

        # estimate beta_inner, beta_outer from turning radius
        if (radius >= 0.0):  # turning left, positive radius, left wheel is inner right wheel is outer
            beta_l = constants.beta_slip_inner_coefficients_left[0]*np.exp(
                constants.beta_slip_inner_coefficients_left[1]*radius)
            v_enc_l += beta_l
            beta_r = constants.beta_slip_outer_coefficients_left[0]*np.exp(
                constants.beta_slip_outer_coefficients_left[1]*radius)
            v_enc_r += beta_r

        else:  # turning right, negative radius, left wheel is outer right is inner
            beta_r = constants.beta_slip_inner_coefficients_right[0]*np.exp(
                constants.beta_slip_inner_coefficients_right[1]*radius)
            v_enc_r += beta_r
            beta_l = constants.beta_slip_outer_coefficients_right[0]*np.exp(
                constants.beta_slip_outer_coefficients_right[1]*radius)
            v_enc_l += beta_l

        qd_comp = np.zeros(2)
        qd_comp[0] = 1/constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1/constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r, radius

    def computeLongSlipCompensationNN(self, v, omega, qd_des, constants):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)
        #
        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS * qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS * qd_des[1]
        beta_l = self.model_beta_l.predict(qd_des)
        beta_r = self.model_beta_r.predict(qd_des)

        v_enc_l += beta_l
        v_enc_r += beta_r

        qd_comp = np.zeros(2)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r, radius

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        self.pub_des_jstate.publish(msg)  # publish in /commands

        # trigger simulators
        # if self.SIMULATOR == 'biral':  # TODO implement torque control
        self.tracked_vehicle_simulator.simulateOneStep(
            qd_des[0], qd_des[1])
        pose, pose_der = self.tracked_vehicle_simulator.getRobotState()
        # fill in base state
        self.basePoseW[:2] = pose[:2]
        self.basePoseW[self.u.sp_crd["AZ"]] = pose[2]
        self.euler = self.u.angPart(self.basePoseW)
        self.baseTwistW[:2] = pose_der[:2]
        self.baseTwistW[self.u.sp_crd["AZ"]] = pose_der[2]

        self.quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.euler))
        self.b_R_w = self.math_utils.rpyToRot(self.euler)
        # publish TF for rviz
        self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), '/base_link', '/world')
        # this is to publish on the topic groundtruth if somebody needs it
        self.pub_odom_msg(self.groundtruth_pub)
        self.q = q_des.copy()
        self.qd = qd_des.copy()
        # this publishes q = q_des, it is just for rviz
        self.joint_pub.publish(msg)

        if np.mod(self.time, 1) == 0:
            print(colored(f"TIME: {self.time}", "red"))

    def pub_odom_msg(self, odom_publisher):
        msg = Odometry()
        msg.pose.pose.orientation.x = self.quaternion[0]
        msg.pose.pose.orientation.y = self.quaternion[1]
        msg.pose.pose.orientation.z = self.quaternion[2]
        msg.pose.pose.orientation.w = self.quaternion[3]
        msg.pose.pose.position.x = self.basePoseW[self.u.sp_crd["LX"]]
        msg.pose.pose.position.y = self.basePoseW[self.u.sp_crd["LY"]]
        msg.pose.pose.position.z = self.basePoseW[self.u.sp_crd["LZ"]]
        msg.twist.twist.linear.x = self.baseTwistW[self.u.sp_crd["LX"]]
        msg.twist.twist.linear.y = self.baseTwistW[self.u.sp_crd["LY"]]
        msg.twist.twist.linear.z = self.baseTwistW[self.u.sp_crd["LZ"]]
        msg.twist.twist.angular.x = self.baseTwistW[self.u.sp_crd["AX"]]
        msg.twist.twist.angular.y = self.baseTwistW[self.u.sp_crd["AY"]]
        msg.twist.twist.angular.z = self.baseTwistW[self.u.sp_crd["AZ"]]
        odom_publisher.publish(msg)


def talker(p, p1):
    launchFileGeneric(rospkg.RosPack().get_path(
            'tractor_description') + "/launch/rviz_nojoints.launch")
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    # initialize properly vars for only 2 actuators (other 2 are caster wheels)
    p.robot.na = 2
    p.initVars()
    p.q_old = np.zeros(2)
    p.initSubscribers()
    p.startupProcedure()

    # init joints
    p.q_des = np.copy(p.q_des_q0)
    p.q_old = np.zeros(2)
    ros.sleep(1.)
    #
    p.q_des = np.zeros(2)
    p.qd_des = np.zeros(2)
    p.tau_ffwd = np.zeros(2)

    p1.start()
    p1.startSimulator()
    p1.loadModelAndPublishers()
    # initialize properly vars for only 2 actuators (other 2 are caster wheels)
    p1.robot.na = 2
    p1.initVars()
    p1.q_old = np.zeros(2)
    p1.initSubscribers()
    p1.startupProcedure()

    # init joints
    p1.q_des = np.copy(p.q_des_q0)
    p1.q_old = np.zeros(2)
    ros.sleep(1.)
    #
    p1.q_des = np.zeros(2)
    p1.qd_des = np.zeros(2)
    p1.tau_ffwd = np.zeros(2)

    while not ros.is_shutdown():
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p1.send_des_jstate(p1.q_des, p1.qd_des, p1.tau_ffwd)

        # p.ros_pub.publishVisual(delete_markers=False)
        # p1.ros_pub.publishVisual(delete_markers=False)


if __name__ == '__main__':
    p = GenericSimulator(robotName+"0")
    p1 = GenericSimulator(robotName+"1")
    try:
        talker(p, p1)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
    ros.signal_shutdown("killed")
    p.deregister_node()
    p1.deregister_node()
    print("Plotting")
    p.plotData()
    p1.plotData()
