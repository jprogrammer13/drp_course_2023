# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from base_controllers.base_controller import BaseController
from base_controllers.utils.common_functions import plotFrameLinear, plotJoint, sendStaticTransform, launchFileGeneric, launchFileNode
import params as conf
import os
import rospkg
from numpy import nan
from matplotlib import pyplot as plt
from base_controllers.utils.ros_publish import RosPub
from  base_controllers.tracked_robot.utils import constants as constants
from base_controllers.tracked_robot.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
from base_controllers.tracked_robot.velocity_generator import VelocityGenerator
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
import catboost as cb
import pinocchio as pin
from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator import TrackedVehicleSimulator, Ground
from base_controllers.utils.common_functions import getRobotModelFloating
from base_controllers.utils.common_functions import checkRosMaster

robotName = "tractor" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        print("Initialized tractor multiple controller---------------------------------------------------------------")
        self.ControlType = 'OPEN_LOOP' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.SAVE_BAGS = False
        self.LONG_SLIP_COMPENSATION = 'NONE'#'NN', 'EXP', 'NONE'

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.comTwistsW = np.zeros(6)
        self.q = np.zeros(2)
        self.qd = np.zeros(2)
        self.tau = np.zeros(2)
        self.q_des = np.zeros(2)
        self.quaternion = np.array([0., 0., 0., 1.])  # fundamental otherwise receivepose gets stuck
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(2)
        self.tau_ffwd = np.zeros(2)
        self.b_R_w = np.eye(3)
        self.time = np.zeros(1)
        self.log_counter = 0

        # log vars
        self.basePoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comPoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_des_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_des_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.qd_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_ffwd_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.tau_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.time_log = np.full((conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        ## add your variables to initialize here
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V= 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(2)
        self.ctrl_v_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.ctrl_omega_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.v_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.omega_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_dot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.des_x = 0.
        self.des_y = 0.
        self.des_theta = 0.
        self.beta_l= 0.
        self.beta_r= 0.
        self.alpha= 0.
        self.alpha_control= 0.
        self.radius = 0.
        self.beta_l_control = 0.
        self.beta_r_control = 0.

        self.state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.beta_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.radius_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_l_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

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
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                self.ctrl_v_log[self.log_counter] = self.ctrl_v
                self.ctrl_omega_log[self.log_counter] = self.ctrl_omega
                self.v_d_log[self.log_counter] = self.v_d
                self.omega_d_log[self.log_counter] = self.omega_d
                self.V_log[self.log_counter] = self.V
                self.V_dot_log[self.log_counter] = self.V_dot
                self.des_state_log[0, self.log_counter] = self.des_x
                self.des_state_log[1, self.log_counter] = self.des_y
                self.des_state_log[2, self.log_counter] = self.des_theta
                self.state_log[0, self.log_counter] = self.basePoseW[self.u.sp_crd["LX"]]
                self.state_log[1, self.log_counter] = self.basePoseW[self.u.sp_crd["LY"]]
                self.state_log[2, self.log_counter] =  self.basePoseW[self.u.sp_crd["AZ"]]

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

        # launch roscore
        checkRosMaster()
        ros.sleep(1.5)
        # run robot state publisher + load robot description + rviz
        #launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/multiple_robots.launch")
        groundParams = Ground()
        self.tracked_vehicle_simulator = TrackedVehicleSimulator(dt=conf.robot_params[p.robot_name]['dt'], ground=groundParams)
        self.tracked_vehicle_simulator.initSimulation(vbody_init=np.array([0,0,0.0]),
                                                      pose_init=np.array([conf.robot_params[p.robot_name]['spawn_x'],
                                                                          conf.robot_params[p.robot_name]['spawn_y'],
                                                                          conf.robot_params[p.robot_name]['spawn_yaw']]))


        # instantiating additional publishers
        self.joint_pub = ros.Publisher("/" + self.robot_name + "/joint_states", JointState, queue_size=1)
        self.groundtruth_pub = ros.Publisher("/" + self.robot_name + "/ground_truth", Odometry, queue_size=1, tcp_nodelay=True)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)


    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        os.system("killall rosmaster rviz")
        super().deregister_node()

    def startupProcedure(self):
            self.basePoseW[2] = conf.robot_params[p.robot_name]['spawn_z']
            self.broadcast_world = False
            self.slow_down_factor = 1
            # loop frequency
            self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))


    def plotData(self):
        if conf.plotting:
            #xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :], p.des_state_log[1, :], "-r", label="desired")
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


            #plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            #joint velocities with limits
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.qd_log[0,:], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[0, :], "-r",  linewidth=4)
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_L")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.qd_log[1, :], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[1, :], "-r",  linewidth=4)                
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_R")
            plt.grid(True)

            #states plot
            plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log)
            plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=np.vstack((p.baseTwistW_log[:2,:],p.baseTwistW_log[5,:])))


            #slippage vars
            plt.figure()
            plt.subplot(4, 1, 1)
            plt.plot(self.time_log, self.beta_l_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_l_control_log, "-r", label="control")
            plt.ylabel("beta_l")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 2)
            plt.plot(self.time_log, self.beta_r_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_r_control_log, "-r", label="control")
            plt.ylabel("beta_r")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 3)
            plt.plot(self.time_log, self.alpha_log, "-b", label="real")
            plt.plot(self.time_log, self.alpha_control_log, "-r", label="control")
            plt.ylabel("alpha")
            plt.ylim([-1, 1])
            plt.grid(True)
            plt.legend()
            plt.subplot(4, 1, 4)
            plt.plot(self.time_log, self.radius_log, "-b")
            plt.ylim([-1,1])
            plt.ylabel("radius")
            plt.grid(True)

            if p.ControlType == 'CLOSED_LOOP':
                # tracking errors
                self.log_e_x, self.log_e_y, self.log_e_theta = self.controller.getErrors()
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(np.sqrt(np.power(self.log_e_x,2) +np.power(self.log_e_y,2)), "-b")
                plt.ylabel("exy")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(self.log_e_theta, "-b")
                plt.ylabel("eth")
                plt.grid(True)

    def mapToWheels(self, v_des,omega_des):
        #
        # # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)
        qd_des = np.zeros(2)
        qd_des[0] = (v_des - omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # left front
        qd_des[1] = (v_des + omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # right front
        return qd_des


    def generateWheelTraj(self, wheel_l = -4.5):
        ####################################
        # OPEN LOOP wl , wr (from -4.5 to 4.5)
        ####################################
        wheel_l_vec = []
        wheel_r_vec = []
        change_interval = 3.
        nsamples = 12
        if wheel_l <= 0.: #this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY, nsamples)
        else:
            wheel_r = np.linspace(constants.MAXSPEED_RADS_PULLEY, -constants.MAXSPEED_RADS_PULLEY, nsamples)
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
        return wheel_l_vec,wheel_r_vec

    def generateOpenLoopTraj(self, R_initial= 0.05, R_final=0.6, increment=0.025, dt = 0.005, long_v = 0.1, direction="left"):
        # only around 0.3
        change_interval = 6.
        increment = increment
        turning_radius_vec = np.arange(R_initial, R_final, increment)
        if direction=='left':
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

    def estimateSlippages(self,W_baseTwist, theta, qd):

        wheel_L = qd[0]
        wheel_R = qd[1]
        w_vel_xy = np.zeros(2)
        w_vel_xy[0] = W_baseTwist[self.u.sp_crd["LX"]]
        w_vel_xy[1] = W_baseTwist[self.u.sp_crd["LY"]]
        omega = W_baseTwist[self.u.sp_crd["AZ"]]

        #compute BF velocity
        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
        b_vel_xy = (w_R_b.T).dot(w_vel_xy)
        b_vel_x = b_vel_xy[0]

        # track velocity  from encoder
        v_enc_l = constants.SPROCKET_RADIUS *  wheel_L
        v_enc_r = constants.SPROCKET_RADIUS *  wheel_R
        B = constants.TRACK_WIDTH

        v_track_l = b_vel_x - omega* B / 2
        v_track_r = b_vel_x + omega* B / 2
        
        # discrepancy bw what it turn out to be (real track) and what it
        # should be (desired) from encoder
        beta_l = v_enc_l-v_track_l
        beta_r = v_enc_r-v_track_r  
        if (abs(b_vel_xy[1])<0.001) or (abs(b_vel_xy[0])<0.001):
            side_slip = 0.
        else:
            side_slip = math.atan2(b_vel_xy[1],b_vel_xy[0])

        return beta_l, beta_r, side_slip


    def computeLongSlipCompensation(self, v, omega, qd_des, constants):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)

        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        #compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS*qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS*qd_des[1]

        #estimate beta_inner, beta_outer from turning radius
        if(radius >= 0.0): # turning left, positive radius, left wheel is inner right wheel is outer
            beta_l = constants.beta_slip_inner_coefficients_left[0]*np.exp(constants.beta_slip_inner_coefficients_left[1]*radius)
            v_enc_l+=beta_l
            beta_r = constants.beta_slip_outer_coefficients_left[0]*np.exp(constants.beta_slip_outer_coefficients_left[1]*radius)
            v_enc_r+=beta_r

        else:# turning right, negative radius, left wheel is outer right is inner
            beta_r = constants.beta_slip_inner_coefficients_right[0]*np.exp(constants.beta_slip_inner_coefficients_right[1]*radius)
            v_enc_r+=beta_r
            beta_l =  constants.beta_slip_outer_coefficients_right[0]*np.exp(constants.beta_slip_outer_coefficients_right[1]*radius)
            v_enc_l+=beta_l

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
        self.pub_des_jstate.publish(msg) #publish in /commands

        #trigger simulators
        self.tracked_vehicle_simulator.simulateOneStep(qd_des[0], qd_des[1])
        pose, pose_der =  self.tracked_vehicle_simulator.getRobotState()
        #fill in base state
        self.basePoseW[:2] = pose[:2]
        self.basePoseW[self.u.sp_crd["AZ"]] = pose[2]
        self.euler = self.u.angPart(self.basePoseW)
        self.baseTwistW[:2] = pose_der[:2]
        self.baseTwistW[self.u.sp_crd["AZ"]] = pose_der[2]

        self.quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.euler))
        self.b_R_w = self.math_utils.rpyToRot(self.euler)

        #publish TF for rviz TODO it is very slow, consider using tf2_ros instead of tf
        self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), "/" + self.robot_name + '/base_link', '/world')
        self.pub_odom_msg(self.groundtruth_pub) #this is to publish on the topic groundtruth if somebody needs it
        self.q = q_des.copy()
        self.qd = qd_des.copy()
        self.joint_pub.publish(msg)  # this publishes q = q_des, it is just for rviz

        if np.mod(self.time,1) == 0:
            print(colored(f"TIME: {self.time}","red"))

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
            'tractor_description') + "/launch/multiple_robots.launch")

    p.start()
    p.startSimulator()
    p.initVars()
    p.startupProcedure()

    # TODO
    # p1.start()
    # p1.startSimulator()
    # p1.initVars()
    # p1.startupProcedure()

    robot_state = Robot()
    if p.SAVE_BAGS:
        bag_name = f"{p.ControlType}_Long_{p.LONG_SLIP_COMPENSATION}.bag"
        p.recorder = RosbagControlledRecorder(bag_name=bag_name)
        p.recorder.start_recording_srv()
    # OPEN loop control
    if p.ControlType == 'OPEN_LOOP':
        counter = 0
        v_ol = np.linspace(0.4, 0.4, np.int32(20./conf.robot_params[p.robot_name]['dt']))
        omega_ol = np.linspace(0.2, 0.2, np.int32(20./conf.robot_params[p.robot_name]['dt']))
        traj_length = len(v_ol)
        p.traj = Trajectory(ModelsList.UNICYCLE, 0, 0, 0, DT=conf.robot_params[p.robot_name]['dt'], v=v_ol, omega=omega_ol)

        while not ros.is_shutdown():
            if counter<traj_length:
                p.v_d = v_ol[counter]
                p.omega_d = omega_ol[counter]
                p.qd_des = p.mapToWheels(p.v_d, p.omega_d )
                counter+=1
            else:
                print(colored("Open loop test accomplished", "red"))
                break

            p.tau_ffwd = np.zeros(2)
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']

            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, _ = p.traj.evalTraj(p.time)
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            #TODO
            #p1.send_des_jstate(p1.q_des, p1.qd_des, p1.tau_ffwd)


            p.ros_pub.publishVisual(delete_markers=False)
            p.beta_l, p.beta_r, p.alpha = p.estimateSlippages(p.baseTwistW, p.basePoseW[p.u.sp_crd["AZ"]], p.qd)

            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999
    else:

        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=40.,    DT=conf.robot_params[p.robot_name]['dt'])
        # initial_des_x = 0.1
        # initial_des_y = 0.1
        # initial_des_theta = 0.3
        initial_des_x = 0.0
        initial_des_y = 0.0
        initial_des_theta = 0.0

        v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.1, omega_max_=0.3)
        p.traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, DT=conf.robot_params[p.robot_name]['dt'],
                            v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)


        # Lyapunov controller parameters
        params = LyapunovParams(K_P=5., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt'])
        p.controller = LyapunovController(params=params)
        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
            robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
            robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, traj_finished = p.traj.evalTraj(p.time)
            if traj_finished:
                break

            if p.ControlType=='CLOSED_LOOP_SLIP_0':
                p.ctrl_v, p.ctrl_omega,  p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished, approx=True)
                p.des_theta -=  p.controller.alpha_exp(p.v_d, p.omega_d)  # we track theta_d -alpha_d

            if p.ControlType == 'CLOSED_LOOP_SLIP':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,approx=False)
                p.des_theta -= p.controller.alpha_exp(p.v_d, p.omega_d)  # we track theta_d -alpha_d

            if p.ControlType=='CLOSED_LOOP_UNICYCLE':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)
            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

            if not p.ControlType=='CLOSED_LOOP_UNICYCLE' and p.LONG_SLIP_COMPENSATION != 'NONE' and not traj_finished:
                if p.LONG_SLIP_COMPENSATION=='NN':
                    p.qd_des, p.beta_l_control, p.beta_r_control, p.radius = p.computeLongSlipCompensationNN(p.ctrl_v, p.ctrl_omega,p.qd_des, constants)
                else:#exponential
                    p.qd_des, p.beta_l_control, p.beta_r_control, p.radius = p.computeLongSlipCompensation(p.ctrl_v, p.ctrl_omega, p.qd_des, constants)
    
            # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            # TODO
            # p1.send_des_jstate(p1.q_des, p1.qd_des, p1.tau_ffwd)

            p.ros_pub.publishVisual(delete_markers=False)

            p.beta_l, p.beta_r, p.alpha = p.estimateSlippages(p.baseTwistW,p.basePoseW[p.u.sp_crd["AZ"]], p.qd)
            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()

if __name__ == '__main__':
    p = GenericSimulator("tractor")
    p1 = GenericSimulator("tractor1")
    try:
        talker(p, p1)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
    ros.signal_shutdown("killed")
    p.deregister_node()
    print("Plotting")
    p.plotData()


