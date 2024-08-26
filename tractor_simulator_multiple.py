# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
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
from base_controllers.utils.math_tools import unwrap_angle
from matplotlib import pyplot as plt
from numpy import nan
import rospkg
import os
import params as conf
import pandas as pd
from base_controllers.utils.common_functions import plotFrameLinear, plotJoint, sendStaticTransform, launchFileGeneric, launchFileNode
from base_controllers.base_controller import BaseController
import rospy as ros
from base_controllers.utils.math_tools import *
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# custom lib
from trajectory import LoopTrajectory
from groundmap import GroundMap
from wls_estimator import *

np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=1000, suppress=True)


robotName = "tractor"  # needs to inherit BaseController


class GenericSimulator(BaseController):

    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf=conf)
        print("Initialized tractor multiple controller---------------------------------------------------------------")
        # 'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0'
        self.ControlType = 'CLOSED_LOOP_SLIP_0'
        self.SAVE_BAGS = False
        self.LONG_SLIP_COMPENSATION = 'WLS'  # 'NN' 'NONE', "WLS"
        self.DEBUG = False
        self.t_start = 0.0
        self.pose_init = None

        self.map_slippage_local_wls = MapSlippageLocalWLSEstimator(
            3, 3, self.robot_name)
        self.map_slippage_global_wls = MapSlippageDistributedWLSEstimator(3, 3)

        self.local_msg = None
        self.global_msg = []

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

        # wls variables
        self.data = pd.DataFrame(
            columns=['wheel_l', 'wheel_r', 'beta_l', 'beta_r', 'alpha', 'i', 'j'])

        # regressor
        self.regressor_beta_l = cb.CatBoostRegressor()
        self.regressor_beta_r = cb.CatBoostRegressor()
        self.regressor_alpha = cb.CatBoostRegressor()
        # laod model
        try:
            if self.LONG_SLIP_COMPENSATION == "NN":
                self.model_beta_l = self.regressor_beta_l.load_model(
                    os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_beta_l.cb')
                self.model_beta_r = self.regressor_beta_r.load_model(
                    os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_beta_r.cb')
                self.model_alpha = self.regressor_alpha.load_model(
                    os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_alpha.cb')
            elif self.LONG_SLIP_COMPENSATION == "WLS":
                # Initialize only the structure, then update the weights
                self.model_beta_l = Model()
                self.model_beta_r = Model()
                self.model_alpha = Model()

        except:
            print(colored(
                "need to generate the models with running tracked_robot/regressor/model_slippage_updated.py", "red"))

        if self.DEBUG:
            vel_gen = VelocityGenerator(
                simulation_time=20., DT=conf.robot_params[self.robot_name]['dt'])
            v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(
                v_max_=0.2, omega_max_=0.3)
            self.traj = Trajectory(ModelsList.UNICYCLE, self.pose_init[0], self.pose_init[1], self.pose_init[2], DT=conf.robot_params[self.robot_name]['dt'],
                                   v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)

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

        # groundParams = Ground()
        self.tracked_vehicle_simulator = TrackedVehicleSimulator(
            # , ground=groundParams)
            dt=conf.robot_params[self.robot_name]['dt'])
        self.tracked_vehicle_simulator.initSimulation(vbody_init=np.array([0, 0, 0.0]),
                                                      pose_init=self.pose_init)

        print("init simulation 1", self.basePoseW)

        self.basePoseW[self.u.sp_crd["LX"]] = self.pose_init[0]
        self.basePoseW[self.u.sp_crd["LY"]] = self.pose_init[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.pose_init[2]

        print("init simulation 2", self.basePoseW)

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
            # xy plot
            plt.figure()
            plt.title(f'{self.robot_name}')
            plt.plot(self.des_state_log[0, :],
                     self.des_state_log[1, :], "-r", label="desired")
            plt.plot(self.state_log[0, :],
                     self.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            # desired command plot
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.title(f'{self.robot_name}')
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
            plt.title(f'{self.robot_name}')
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
            # base position
            plotFrameLinear(name='position', title=f'{self.robot_name}', time_log=self.time_log,
                            des_Pose_log=self.des_state_log, Pose_log=self.state_log)            # base velocity
            # plotFrameLinear(name='velocity', title=f'{self.robot_name}', time_log=self.time_log, Twist_log=np.vstack(
            #     (self.baseTwistW_log[:2, :], self.baseTwistW_log[5, :])))

            # slippage vars
            plt.figure()
            plt.subplot(3, 1, 1)
            plt.plot(self.time_log, self.beta_l_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_l_control_log, "-r", label="control")
            plt.ylabel("beta_l")
            plt.legend()
            plt.grid(True)
            plt.subplot(3, 1, 2)
            plt.plot(self.time_log, self.beta_r_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_r_control_log, "-r", label="control")
            plt.ylabel("beta_r")
            plt.legend()
            plt.grid(True)
            plt.subplot(3, 1, 3)
            plt.plot(self.time_log, self.alpha_log, "-b", label="real")
            plt.plot(self.time_log, self.alpha_control_log, "-r", label="control")
            plt.ylabel("alpha")
            plt.ylim([-0.4, 0.4])
            plt.grid(True)
            plt.legend()

            # tracking errors
            self.log_e_x, self.log_e_y, self.log_e_theta = self.controller.getErrors()
            plt.figure()
            plt.subplot(2, 1, 1)
            exy = np.sqrt(np.power(self.log_e_x, 2) + np.power(self.log_e_y, 2))
            plt.plot(exy, "-b")
            plt.ylabel("exy")
            plt.title("tracking error")
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

    def estimateSlippages(self, W_baseTwist, theta, qd):

        wheel_L = qd[0] + np.random.normal()*1e-4
        wheel_R = qd[1] + np.random.normal()*1e-4
        w_vel_xy = np.zeros(2)
        w_vel_xy[0] = W_baseTwist[self.u.sp_crd["LX"]] + \
            (np.random.normal()*1e-4)
        w_vel_xy[1] = W_baseTwist[self.u.sp_crd["LY"]] + \
            (np.random.normal()*1e-4)
        omega = W_baseTwist[self.u.sp_crd["AZ"]] + (np.random.normal()*1e-4)

        # compute BF velocity
        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
        b_vel_xy = (w_R_b.T).dot(w_vel_xy)
        v = np.linalg.norm(b_vel_xy)

        # compute turning radius for logging
        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

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

        return beta_l, beta_r, side_slip, radius

    def computeLongSlipCompensationNN(self, qd_des, constants):
        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS * qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS * qd_des[1]
        # predict the betas from NN
        beta_l = self.model_beta_l.predict(qd_des)
        beta_r = self.model_beta_r.predict(qd_des)

        v_enc_l += beta_l
        v_enc_r += beta_r

        qd_comp = np.zeros(2)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r

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
        if np.any(qd_des > np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])) or np.any(qd_des < -np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])):
            print(colored("wheel speed beyond limits, NN might do wrong predictions", "red"))
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
        self.beta_l, self.beta_r, self.alpha, self.radius = self.estimateSlippages(
            self.baseTwistW, self.basePoseW[self.u.sp_crd["AZ"]], self.qd)
        # log variables
        self.logData()

        # this is for getting the slipage
        return self.qd[0], self.qd[1], self.beta_l, self.beta_r, self.alpha

    def getGPSReading(self):
        return self.basePoseW + self.noise_pose, self.baseTwistW + self.noise_twist

    def pub_odom_msg(self, odom_publisher):
        msg = Odometry()
        msg.header.stamp = ros.Time.from_sec(self.time)
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


def start_robots(n_robots, robots, trajectory, groundMap):
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
        robot.t_start = t
        # if robot.DEBUG:
        #     t += 1
        # else:
        #     t += np.random.randint(5, 10)
        t += np.random.randint(5, 10)

        robot.global_msg = [None for _ in range(n_robots)]

        x, y, robot.old_theta, _, _, _, _ = trajectory.eval_trajectory(
            robot.t_start)

        print(
            f"robot: {robot.robot_name}, t_start: {robot.t_start} x: {x} y: {y} yaw: {robot.old_theta}")
        robot.set_pose_init(x, y, robot.old_theta)

        robot.initVars()
        robot.start()
        robot.startSimulator()

        ground = groundMap.get_ground(x, y)
        robot.tracked_vehicle_simulator.setGround(ground)
        # DEBUG
        # robot.tracked_vehicle_simulator.NO_SLIPPAGE = True

        robot.startupProcedure()
        robot.robot_state = Robot()

        # Lyapunov controller parameters
        robot.controller = LyapunovController(params=params)


def generate_path_msg(trajectory):

    t = np.linspace(0, trajectory.t_tot, 100)
    x, y, _, _, _, _, _ = trajectory.eval_trajectory(t)

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


def generate_grid_msg(groundMap, data_path):
    friction_coeff_matrix = np.array([[groundMap.map[i][j].friction_coefficient for j in range(
        groundMap.j_max+1)] for i in range(groundMap.i_max+1)])

    print(friction_coeff_matrix)

    plt.matshow(friction_coeff_matrix, vmin=0, cmap='Greys_r')
    plt.colorbar()
    plt.savefig(f'{data_path}/grid_friction.png')

    # Normalize friction coefficients to the range 0-100 for OccupancyGrid (0-100 indicates probability, -1 is unknown)
    normalized_friction = (friction_coeff_matrix *
                           100).astype(np.int8).flatten()

    # Create the OccupancyGrid message
    grid_msg = OccupancyGrid()

    # Set up the header
    grid_msg.header = Header()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "world"
    # Set up the info
    grid_msg.info = MapMetaData()
    grid_msg.info.resolution = 3.0  # Each grid cell is 1x1 meter, adjust as necessary
    # grid_msg.info.resolution = 1.0  # Each grid cell is 1x1 meter, adjust as necessary
    grid_msg.info.width = int(groundMap.width // 3)
    grid_msg.info.height = int(groundMap.height // 3)

    # Set the origin of the grid map
    grid_msg.info.origin = Pose()
    grid_msg.info.origin.position.x = groundMap.x_min
    grid_msg.info.origin.position.y = groundMap.y_min
    grid_msg.info.origin.position.z = 0
    grid_msg.info.origin.orientation.w = 1.0

    # Set the data
    grid_msg.data = normalized_friction.tolist()

    return grid_msg


def talker(n_robots, robots, trajectory, groundMap, data_path):
    global df
    # common stuff

    start_robots(n_robots, robots, trajectory, groundMap)
    # debug
    # robots[0].set_pose_init(0,0,0)

    path_pub = ros.Publisher('/trajectory_path', Path, queue_size=10)
    path_msg = generate_path_msg(trajectory)

    grid_pub = rospy.Publisher(
        "/ground_friction_grid", OccupancyGrid, queue_size=10)
    grid_msg = generate_grid_msg(groundMap, data_path)

    slow_down_factor = 1
    # loop frequency
    rate = ros.Rate(1 / (slow_down_factor * conf.global_dt))

    time_global = 0.

    # previous traj
    # vel_gen = VelocityGenerator(simulation_time=40., DT=conf.robot_params[robots[0].robot_name]['dt'])
    # initial_des_x = 0.0
    # initial_des_y = 0.0
    # initial_des_theta = 0.0
    # v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.1, omega_max_=0.3)
    # robots[0].traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta,
    #                     DT=conf.robot_params[robots[0].robot_name]['dt'],  v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)
    # robots[0].traj.set_initial_time(start_time=time_global)

    # CLOSE loop control
    while not ros.is_shutdown():

        path_pub.publish(path_msg)
        grid_pub.publish(grid_msg)

        for robot in robots:

            robot.time = time_global
            # update kinematics
            robot.robot_state.x = robot.basePoseW[robot.u.sp_crd["LX"]]
            robot.robot_state.y = robot.basePoseW[robot.u.sp_crd["LY"]]
            robot.robot_state.theta = robot.basePoseW[robot.u.sp_crd["AZ"]]
            # print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            # DEBUG uniform friction coeff
            if robot.DEBUG:
                ground = Ground(friction_coefficient=0.1)
            else:
                # update the ground according on the position
                ground = groundMap.get_ground(
                    robot.robot_state.x, robot.robot_state.y)
            robot.tracked_vehicle_simulator.setGround(ground)

            i, j = groundMap.coords_to_index(
                robot.robot_state.x, robot.robot_state.y)
            if robot.LONG_SLIP_COMPENSATION == "WLS":
                wls_global_regressor = robot.map_slippage_global_wls.map_wls_regressors[i][j]

                robot.model_beta_l.theta = wls_global_regressor.theta[..., 0]
                robot.model_beta_r.theta = wls_global_regressor.theta[..., 1]
                robot.model_alpha.theta = wls_global_regressor.theta[..., 2]

            if robot.DEBUG:
                robot.des_x, robot.des_y, robot.des_theta, robot.v_d, robot.omega_d, robot.v_dot_d, robot.omega_dot_d, traj_finished = robot.traj.evalTraj(
                    robot.time)
                if traj_finished:
                    break
            else:
                robot.des_x, robot.des_y, robot.des_theta, robot.v_d, robot.omega_d, robot.v_dot_d, robot.omega_dot_d = trajectory.eval_trajectory(
                    robot.time + robot.t_start)
                robot.des_theta, robot.old_theta = unwrap_angle(
                    robot.des_theta, robot.old_theta)

            if robot.ControlType == 'CLOSED_LOOP_SLIP_0':
                robot.ctrl_v, robot.ctrl_omega, robot.V, robot.V_dot, robot.alpha_control = robot.controller.control_alpha(
                    robot.robot_state, robot.time, robot.des_x, robot.des_y, robot.des_theta, robot.v_d, robot.omega_d, robot.v_dot_d, robot.omega_dot_d, False, robot.model_alpha, approx=True)

            if robot.ControlType == 'CLOSED_LOOP_UNICYCLE':
                robot.ctrl_v, robot.ctrl_omega, robot.V, robot.V_dot = robot.controller.control_unicycle(
                    robot.robot_state, robot.time, robot.des_x, robot.des_y, robot.des_theta, robot.v_d, robot.omega_d, False)

            robot.qd_des = robot.mapToWheels(robot.ctrl_v, robot.ctrl_omega)

            if not robot.ControlType == 'CLOSED_LOOP_UNICYCLE':
                if robot.LONG_SLIP_COMPENSATION == 'NN' or robot.LONG_SLIP_COMPENSATION == 'WLS':
                    robot.qd_des, robot.beta_l_control, robot.beta_r_control = robot.computeLongSlipCompensationNN(
                        robot.qd_des, constants)

            # # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # # senting it to be tracked from the impedance loop
            robot.q_des = robot.q_des + robot.qd_des * \
                conf.robot_params[robot.robot_name]['dt']

            wheel_l, wheel_r, beta_l, beta_r, alpha = robot.send_des_jstate(
                robot.q_des, robot.qd_des, robot.tau_ffwd)

            # save data with low freq
            if time_global % 0.5 == 0 and time_global > 5:
                observation = pd.DataFrame({
                    'wheel_l': [wheel_l],
                    'wheel_r': [wheel_r],
                    'beta_l': [beta_l],
                    'beta_r': [beta_r],
                    'alpha': [alpha],
                    'i': [i],
                    'j': [j],
                })

                robot.data = pd.concat(
                    [robot.data, observation], ignore_index=True)

            # Estimate local regressor
            if time_global % 5 == 0 and time_global != 0:
                print(f"{robot.robot_name} computing wls...")
                robot.map_slippage_local_wls.compute_wls_regressor(robot.data)
                robot.local_msg = robot.map_slippage_local_wls.generate_msg()
                # Send data to the other robots in a

        # wait for synconization of the control loop
        rate.sleep()
        # to avoid issues of dt 0.0009999
        time_global = np.round(
            time_global + np.array([conf.global_dt]), 3)

        if np.mod(time_global, 1) == 0:
            print(colored(f"TIME: {time_global}", "red"))

        if time_global % 6 == 0 and time_global != 0:

            # Emulate token-ring comunication
            for i in range((n_robots*2)-1):
                i_ = i % n_robots
                # print(f"tractor{i_}")
                robots[i_].global_msg[i_] = robots[i_].local_msg
                # print(robot.global_msg)
                next_robot_id = (i_+1) % (n_robots)
                robots[next_robot_id].global_msg = robots[i_].global_msg

            # Once everyone has all the local estimates, compute the global
            for i, robot in enumerate(robots):
                print(f"tractor{i} computing global wls...")
                robot.map_slippage_global_wls.compute_wls_regressor(
                    robot.global_msg)


def generate_circle_viapoints(radius, num_points):
    # Generate angles evenly spaced around the circle
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Calculate x and y coordinates for each angle
    x_coords = radius * np.cos(angles)
    y_coords = radius * np.sin(angles)

    # Combine x and y into a single array of shape (num_points, 2)
    viapoints = np.column_stack((x_coords, y_coords))

    return viapoints


def plot_wls(wls_regressor, data, param_str, param_id):
    # Create figure and 3x3 subplots
    fig, axes = plt.subplots(
        3, 3, subplot_kw={'projection': '3d'}, figsize=(12, 12))

    # Set a large title for the entire figure
    fig.suptitle(param_str, fontsize=20)

    # Loop through the 3x3 grid to plot something on each subplot
    for i in range(3):
        for j in range(3):
            ax = axes[i, j]
            df = data[(data.i == i) & (data.j == j)]
            # Example data: plotting a simple surface
            wheel_l, wheel_r = np.meshgrid(np.linspace(df["wheel_l"].min(), df["wheel_l"].max(), 100),
                                           np.linspace(df["wheel_r"].min(), df["wheel_r"].max(), 100))
            theta = wls_regressor.map_wls_regressors[i][j].theta[:, param_id]
            wls = theta[0] + theta[1] * wheel_l + theta[2] * wheel_r
            ax.plot_surface(wheel_l, wheel_r, wls, cmap="viridis")
            ax.scatter(df["wheel_l"], df["wheel_r"], df[param_str],
                       color='red')  # Original data points
            # Optional: adjust fontsize
            ax.set_title(f"Patch {i}:{j}", fontsize=15)
            ax.set_xlabel("wheel_l")
            ax.set_ylabel("wheel_r")
            # Optional: You can label the z-axis if needed
            ax.set_zlabel(param_str)

    # Adjust the layout to be compact with more padding
    # Increase space between subplots
    plt.subplots_adjust(wspace=0.4, hspace=0.4)

    # Adjust layout to avoid overlap of suptitle
    # Adjust rect to accommodate the suptitle
    plt.tight_layout(rect=[0, 0, 1, 1])


if __name__ == '__main__':
    data_path = f"{os.environ.get('LOCOSIM_DIR')}/robot_control/drp_course_2023/data"
    # Enable interactive mode
    groundMap = GroundMap(9, 9, 3)
    # groundMap = GroundMap(6, 6)

    traj_viapoints = np.array([[-4.,  0.5],
                               [-1.5, -3.],
                               [0., -4.],
                               [2., -3.],
                               [4.,  0.5],
                               [3.,  3.],
                               [1.5,  4.],
                               [0.,  2.9],
                               [-2.,  2.5]])

    # traj_viapoints = generate_circle_viapoints(2.5, 20)
    traj_t_tot = 50
    trajectory = LoopTrajectory(traj_viapoints, traj_t_tot)

    n_tractors = 1

    tractors = []

    for i in range(n_tractors):
        tractor = GenericSimulator(f"tractor{i}")
        tractors.append(tractor)
    try:
        talker(n_tractors, tractors, trajectory, groundMap, data_path)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    ros.signal_shutdown("killed")

    df = pd.DataFrame(columns=['wheel_l', 'wheel_r',
                      'beta_l', 'beta_r', 'alpha', 'i', 'j'])
    for tractor in tractors:
        df = pd.concat([tractor.data, df], ignore_index=True)

    # plot_wls(tractors[0].map_slippage_global_wls, df, 'beta_l', 0)
    # plot_wls(tractors[0].map_slippage_global_wls, df, 'beta_r', 1)
    # plot_wls(tractors[0].map_slippage_global_wls, df, 'alpha', 2)

    for tractor in tractors:
        tractor.deregister_node()
        if tractor.DEBUG:
            tractor.plotData()
        tractor.plotData()
