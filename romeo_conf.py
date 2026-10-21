# -*- coding: utf-8 -*-
"""
Configuration file for Assignment 01 (2026): double support and whole-body tasks in TSID biped walking.
"""

import os

import numpy as np
import pinocchio as pin
from example_robot_data.robots_loader import getModelPath

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

# robot parameters
# ----------------------------------------------
urdf = "/romeo_description/urdf/romeo_small.urdf"
path = getModelPath(urdf)
urdf = path + urdf
srdf = path + '/romeo_description/srdf/romeo_small.srdf'
path = os.path.join(path, '../..')

nv = 37
lxp = 0.10  # foot length in positive x direction
lxn = 0.05  # foot length in negative x direction
lyp = 0.05  # foot length in positive y direction
lyn = 0.05  # foot length in negative y direction
lz = 0.07  # foot sole height with respect to ankle joint
mu = 0.3  # friction coefficient
fMin = 0.0  # minimum normal force
fMax = 1e6  # maximum normal force
rf_frame_name = "RAnkleRoll"  # right foot frame name
lf_frame_name = "LAnkleRoll"  # left foot frame name
contactNormal = np.array([0.0, 0.0, 1.0])  # direction of the normal to the contact surface

# tray parameters (part 2 of the assignment)
# ----------------------------------------------
tray = False  # if True the robot carries the tray: its mass is added to the hand and it is drawn in the viewer
hand_frame_name = "RWristPitch"  # frame of the hand carrying the tray
payload_joint = "RWristPitch"  # the payload is attached to the link of this joint
payload_offset = [0.089, 0.0, -0.021]  # position of the payload in the frame of payload_joint (centre of the hand)
payload_mass = 0.5  # mass of the payload when tray = True: empty tray 0.5 kg [kg]

# reference trajectories (CoM, feet, CoP and phases of the gait for 6 steps)
# ----------------------------------------------
DATA_FILE_TSID = "romeo_walking_ds.npz"  # with double support phases
# DATA_FILE_TSID = "romeo_walking_no_ds.npz"  # without double support phases

# configuration for TSID
# ----------------------------------------------
dt = 0.002  # controller time step
T_pre = 1.5  # simulation time before starting to walk
T_post = 1.5  # simulation time after walking
transition_time = 0.0  # duration of the contact force ramp before lift-off (0 = instantaneous)

w_com = 1.0  # weight of center of mass task
w_cop = 0.0  # weight of center of pressure task
w_am = 1e-6  # weight of angular momentum task
w_foot = 1e0  # weight of the foot motion task
w_contact = 1e2  # weight of the foot in contact
w_posture = 1e-2  # weight of joint posture task
w_forceRef = 1e-5  # weight of force regularization task
w_torque_bounds = 1.0  # weight of the torque bounds
w_joint_bounds = 1.0

w_hand = 0.0  # weight of the hand (tray) task
kp_hand = 10.0  # proportional gain of the hand (tray) task
hand_priority = 1  # priority level of the hand task (0 = hard constraint, 1 = cost function)
hand_mask = np.ones(6)  # mask of the hand task: [x, y, z, rot_x, rot_y, rot_z]
hand_local_frame = True  # if True the task (and its mask) is expressed in the hand frame, otherwise in a world-aligned frame

tau_max_scaling = 1.45  # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0  # proportional gain of contact constraint
kp_foot = 10.0  # proportional gain of contact constraint
kp_com = 10.0  # proportional gain of center of mass task
kp_am = 10.0  # proportional gain of angular momentum task
kp_posture = 1.0  # proportional gain of joint posture task
gain_vector = kp_posture * np.ones(nv - 6)
masks_posture = np.ones(nv - 6)

# configuration for viewer
# ----------------------------------------------
try:
    import pinocchio.visualize  # the submodule is not loaded by "import pinocchio"
    viewer = pin.visualize.MeshcatVisualizer
except ImportError:  # meshcat not available: run without viewer
    viewer = None
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viewer every DISPLAY_N time steps
