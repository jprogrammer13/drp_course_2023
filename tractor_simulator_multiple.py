# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
from trajectory import LoopTrajectory
from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.common_functions import getRobotModelFloating
from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator import TrackedVehicleSimulator, Ground
import pinocchio as pin
import catboost as cb
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from termcolor import colored
from base_controllers.tracked_robot.velocity_generator import VelocityGenerator
from base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
from base_controllers.tracked_robot.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from base_controllers.tracked_robot.utils import constants as constants
from base_controllers.utils.ros_publish import RosPub
from matplotlib import pyplot as plt
from numpy import nan
import rospkg
import os
import params as conf
from base_controllers.utils.common_functions import plotFrameLinear, plotJoint, sendStaticTransform, launchFileGeneric, launchFileNode
from base_controllers.base_controller import BaseController
import rospy as ros
from base_controllers.utils.math_tools import *

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=1000, suppress=True)


robotName = "tractor"  # needs to inherit BaseController


class GenericSimulator(BaseController):

    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf=conf)
        print("Initialized tractor multiple controller---------------------------------------------------------------")
        # 'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.ControlType = 'CLOSED_LOOP_UNICYCLE'
        self.SAVE_BAGS = False
        self.LONG_SLIP_COMPENSATION = 'NONE'  # 'NN', 'EXP', 'NONE'
        self.t_start = 0.0
        self.pose_init = None

    def set_pose_init(self, x, y, yaw):
        self.pose_init = np.array([x, y, yaw])

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.comTwistsW = np.zeros(6)
        self.q = np.zeros(2)
        self.qd = np.zeros(2)
        self.tau = np.zeros(2)
        self.q_des = np.zeros(2)
        # fundamental otherwise receivepose gets stuck
        self.quaternion = np.array([0., 0., 0., 1.])
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(2)
        self.tau_ffwd = np.zeros(2)
        self.b_R_w = np.eye(3)
        self.time = np.zeros(1)
        self.log_counter = 0

        # log vars
        self.basePoseW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comPoseW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comTwistW_log = np.full(
            (6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_des_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_des_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_ffwd_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_log = np.full(
            (2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.time_log = np.full(
            (conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        # add your variables to initialize here
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V = 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(2)
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

        # regressor
        self.model = cb.CatBoostRegressor()
        # laod model
        try:
            self.model_beta_l.load_model(os.environ[
                'LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/controllers/regressor/model_beta_l.cb')
            self.model_beta_r.load_model(os.environ[
                'LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/controllers/regressor/model_beta_r.cb')
        except:
            print(colored(
                "need to generate the models with running tracked_robot/controller/regressor/model_slippage_updated.py"))

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

            self.basePoseW_log[:, self.log_counter] = self.basePoseW
            self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:, self.log_counter] = self.q
            self.qd_des_log[:, self.log_counter] = self.qd_des
            self.qd_log[:, self.log_counter] = self.qd
            self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
            self.tau_log[:, self.log_counter] = self.tau
            self.time_log[self.log_counter] = self.time
            self.log_counter += 1

    def startSimulator(self):

        # if self.pose_init != None:
        #     np.array([conf.robot_params[self.robot_name]['spawn_x'],
        #               conf.robot_params[self.robot_name]['spawn_y'],
        #               conf.robot_params[self.robot_name]['spawn_yaw']])

        # run robot state publisher + load robot description + rviz
        # launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/multiple_robots.launch")
        groundParams = Ground()
        self.tracked_vehicle_simulator = TrackedVehicleSimulator(
            dt=conf.robot_params[self.robot_name]['dt'], ground=groundParams)
        self.tracked_vehicle_simulator.initSimulation(vbody_init=np.array([0, 0, 0.0]),
                                                      pose_init=self.pose_init)

        # instantiating additional publishers
        self.joint_pub = ros.Publisher(
            "/" + self.robot_name + "/joint_states", JointState, queue_size=1)
        self.groundtruth_pub = ros.Publisher(
            "/" + self.robot_name + "/ground_truth", Odometry, queue_size=1, tcp_nodelay=True)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.pub_des_jstate = ros.Publisher(
            "/" + self.robot_name + "/command", JointState, queue_size=1, tcp_nodelay=True)

    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        os.system("killall rosmaster rviz")
        super().deregister_node()

    def startupProcedure(self):
        self.basePoseW[2] = conf.robot_params[self.robot_name]['spawn_z']

    def plotData(self):
        if conf.plotting:
            # xy plot
            plt.figure()
            plt.plot(self.des_state_log[0, :],
                     self.des_state_log[1, :], "-r", label="desired")
            plt.plot(self.state_log[0, :],
                     self.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            # # command plot
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time_log, self.ctrl_v_log, "-b", label="REAL")
            plt.plot(self.time_log, self.v_d_log, "-r", label="desired")
            plt.legend()
            plt.ylabel("linear velocity[m/s]")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(self.time_log, self.ctrl_omega_log, "-b", label="REAL")
            plt.plot(self.time_log, self.omega_d_log, "-r", label="desired")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("angular velocity[rad/s]")
            plt.grid(True)

            # plotJoint('position', self.time_log, q_log=self.q_log, q_des_log=self.q_des_log, joint_names=self.joint_names)
            # joint velocities with limits
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time_log, self.qd_log[0, :], "-b",  linewidth=3)
            plt.plot(self.time_log, self.qd_des_log[0, :], "-r",  linewidth=4)
            plt.plot(self.time_log, constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(self.time_log))), "-k",  linewidth=4)
            plt.plot(self.time_log, -constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(self.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_L")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(self.time_log, self.qd_log[1, :], "-b",  linewidth=3)
            plt.plot(self.time_log, self.qd_des_log[1, :], "-r",  linewidth=4)
            plt.plot(self.time_log, constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(self.time_log))), "-k",  linewidth=4)
            plt.plot(self.time_log, -constants.MAXSPEED_RADS_PULLEY *
                     np.ones((len(self.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_R")
            plt.grid(True)

            # states plot
            plotFrameLinear(name='position', time_log=self.time_log,
                            des_Pose_log=self.des_state_log, Pose_log=self.state_log)
            plotFrameLinear(name='velocity', time_log=self.time_log, Twist_log=np.vstack(
                (self.baseTwistW_log[:2, :], self.baseTwistW_log[5, :])))

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

            if self.ControlType == 'CLOSED_LOOP':
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
        return qd_des

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
            time = np.round(time + conf.robot_params[self.robot_name]['dt'], 3)
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
        self.tracked_vehicle_simulator.simulateOneStep(qd_des[0], qd_des[1])
        pose, pose_der = self.tracked_vehicle_simulator.getRobotState()
        # fill in base state
        self.basePoseW[:2] = pose[:2]
        self.basePoseW[self.u.sp_crd["AZ"]] = pose[2]
        self.euler = self.u.angPart(self.basePoseW)
        self.baseTwistW[:2] = pose_der[:2]
        self.baseTwistW[self.u.sp_crd["AZ"]] = pose_der[2]

        self.quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.euler))
        # this will be used only with slopes
        # self.b_R_w = self.math_utils.rpyToRot(self.euler)

        # publish TF for rviz TODO it is very slow, consider using tf2_ros instead of tf
        self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), "/" + self.robot_name + '/base_link', '/world')
        # this is to publish on the topic groundtruth if somebody needs it
        self.pub_odom_msg(self.groundtruth_pub)
        self.q = q_des.copy()
        self.qd = qd_des.copy()
        # this publishes q = q_des, it is just for rviz
        self.joint_pub.publish(msg)

        self.ros_pub.publishVisual(delete_markers=False)
        self.beta_l, self.beta_r, self.alpha = self.estimateSlippages(
            self.baseTwistW, self.basePoseW[self.u.sp_crd["AZ"]], self.qd)
        # log variables
        self.logData()

    def getGPSReading(self):
        return self.basePoseW + self.noise_pose, self.baseTwistW + self.noise_twist

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


def start_robots(robots, trajectory):
    # launch roscore
    checkRosMaster()
    launchFileGeneric(rospkg.RosPack().get_path(
        'tractor_description') + "/launch/multiple_robots.launch")

    params = LyapunovParams(K_P=10., K_THETA=1., DT=conf.global_dt)

    # set seed for random generator
    np.random.seed(13)
    # t_tot = trajectory.t_tot
    t = 0

    for robot in robots:
        t += np.random.randint(5,10)
        robot.t_start = t

        x, y, yaw, _, _, _, _, _ = trajectory.eval_trajectory(robot.t_start)
        print(f"robot: {robot.robot_name}, t_start: {robot.t_start} x: {x} y: {y} yaw: {yaw}")

        robot.set_pose_init(x, y, yaw)

        robot.start()
        robot.startSimulator()
        robot.initVars()
        robot.startupProcedure()
        robot.robot_state = Robot()

        # Lyapunov controller parameters
        robot.controller = LyapunovController(params=params)

def generate_path_msg(trajectory):

    t = np.linspace(0, trajectory.t_tot, 100)
    x, y, _ , _ , _, _, _, _ = trajectory.eval_trajectory(t)

    path_msg = Path()
    path_msg.header.frame_id = 'world'

    for i in range(len(t)):
        pose = PoseStamped()
        pose.header.stamp = ros.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        pose.pose.position.z = 0.0
        # You can also add orientation if available
        path_msg.poses.append(pose)

    return path_msg


def talker(robots, trajectory):
    # common stuff

    start_robots(robots, trajectory)

    path_pub = ros.Publisher('/trajectory_path', Path, queue_size=10)
    path_msg = generate_path_msg(trajectory)

    slow_down_factor = 1
    # loop frequency
    rate = ros.Rate(1 / (slow_down_factor * conf.global_dt))

    time_global = 0.

    # CLOSE loop control

    while not ros.is_shutdown():

        path_pub.publish(path_msg)

        for robot in robots:

            robot.time = time_global
            # update kinematics
            robot.robot_state.x = robot.basePoseW[robot.u.sp_crd["LX"]]
            robot.robot_state.y = robot.basePoseW[robot.u.sp_crd["LY"]]
            robot.robot_state.theta = robot.basePoseW[robot.u.sp_crd["AZ"]]
            # print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            # self.des_x, self.des_y, self.des_theta, self.v_d, self.omega_d, self.v_dot_d, self.omega_dot_d = self.traj.evalTraj(
            #     self.time)
            # if traj_finished:
            #     break

            # if self.ControlType == 'CLOSED_LOOP_UNICYCLE':
            #     self.ctrl_v, self.ctrl_omega, self.V, self.V_dot = self.controller.control_unicycle(
            #         self.robot_state, self.time, self.des_x, self.des_y, self.des_theta, self.v_d, self.omega_d, False)

            # self.qd_des = self.mapToWheels(self.ctrl_v, self.ctrl_omega)

            # if not self.ControlType == 'CLOSED_LOOP_UNICYCLE' and self.LONG_SLIP_COMPENSATION != 'NONE' and not traj_finished:
            #     if self.LONG_SLIP_COMPENSATION == 'NN':
            #         self.qd_des, self.beta_l_control, self.beta_r_control, self.radius = self.computeLongSlipCompensationNN(
            #             self.ctrl_v, self.ctrl_omega, self.qd_des, constants)
            #     else:  # exponential
            #         self.qd_des, self.beta_l_control, self.beta_r_control, self.radius = self.computeLongSlipCompensation(
            #             self.ctrl_v, self.ctrl_omega, self.qd_des, constants)

            # # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # # senting it to be tracked from the impedance loop
            # self.q_des = self.q_des + self.qd_des * conf.robot_params[self.robot_name]['dt']

            robot.send_des_jstate(robot.q_des, robot.qd_des, robot.tau_ffwd)

        # wait for synconization of the control loop
        rate.sleep()
        # to avoid issues of dt 0.0009999
        time_global = np.round(
            time_global + np.array([conf.global_dt]), 3)

        if np.mod(time_global, 1) == 0:
            print(colored(f"TIME: { time_global}", "red"))


if __name__ == '__main__':

    traj_viapoints = np.array([[-0.5, 0], [0, -1.5], [1.5, -2.5], [2.5, -1.5], [
                              3.5, -1.5], [3.5, 2.5], [2.5, 3.5], [1.5, 2.4], [0.5, 1.6]])
    traj_t_tot = 40
    trajectory = LoopTrajectory(traj_viapoints, traj_t_tot)

    n_tracktors = 5
    tracktors = []

    for i in range(n_tracktors):
        tracktor = GenericSimulator(f"tractor{i}")
        tracktors.append(tracktor)
    try:
        talker(tracktors, trajectory)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    ros.signal_shutdown("killed")
    for tracktor in tracktors:
        tracktor.deregister_node()
        print(f"Plotting data of {tracktor.robot_name}")
        # tracktor.plotData()
