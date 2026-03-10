#!/usr/bin/env python3
"""
Module for loading and parsing Digit robot CSV data.
Provides all 191+ columns as named variables.
"""

import numpy as np

# Load the CSV file with headers
digit_data = np.genfromtxt("digit_cropped_data_full.csv", delimiter=",", skip_header=1, filling_values=0.0)
print(f"Loaded digit data: {digit_data.shape[0]} rows, {digit_data.shape[1]} columns")

digit_data = digit_data[0:977, :] # Crop to 977 rows to match timestamp length



# Parse all columns into named variables
timestamp = digit_data[:, 0]

# Base position (1-3)
pos_x = digit_data[:, 1]
pos_y = digit_data[:, 2]
pos_z = digit_data[:, 3]

# Base orientation quaternion (4–7)
quat_w = digit_data[:, 4]
quat_i = digit_data[:, 5]
quat_j = digit_data[:, 6]
quat_k = digit_data[:, 7]

# Base velocity (8-10)
vel_x = digit_data[:, 8]
vel_y = digit_data[:, 9]
vel_z = digit_data[:, 10]

# Angular rates (11-13)
r_rate = digit_data[:, 11]
p_rate = digit_data[:, 12]
y_rate = digit_data[:, 13]

# Joint positions - Left leg (14-19)
left_hip_roll = digit_data[:, 14]
left_hip_yaw = digit_data[:, 15]
left_hip_pitch = digit_data[:, 16]
left_knee = digit_data[:, 17]
left_toe_A = digit_data[:, 18]
left_toe_B = digit_data[:, 19]

# Joint positions - Left arm (20-23)
left_shoulder_roll = digit_data[:, 20]
left_shoulder_pitch = digit_data[:, 21]
left_shoulder_yaw = digit_data[:, 22]
left_elbow = digit_data[:, 23]

# Joint positions - Right leg (24-29)
right_hip_roll = digit_data[:, 24]
right_hip_yaw = digit_data[:, 25]
right_hip_pitch = digit_data[:, 26]
right_knee = digit_data[:, 27]
right_toe_A = digit_data[:, 28]
right_toe_B = digit_data[:, 29]

# Joint positions - Right arm (30-33)
right_shoulder_roll = digit_data[:, 30]
right_shoulder_pitch = digit_data[:, 31]
right_shoulder_yaw = digit_data[:, 32]
right_elbow = digit_data[:, 33]

# Passive joints - Left leg (34-38)
left_shin = digit_data[:, 34]
left_tarsus = digit_data[:, 35]
left_toe_pitch = digit_data[:, 36]
left_toe_roll = digit_data[:, 37]
left_heel_spring = digit_data[:, 38]

# Passive joints - Right leg (39-43)
right_shin = digit_data[:, 39]
right_tarsus = digit_data[:, 40]
right_toe_pitch = digit_data[:, 41]
right_toe_roll = digit_data[:, 42]
right_heel_spring = digit_data[:, 43]

# Joint velocities - Left leg (44-49)
left_hip_roll_vel = digit_data[:, 44]
left_hip_yaw_vel = digit_data[:, 45]
left_hip_pitch_vel = digit_data[:, 46]
left_knee_vel = digit_data[:, 47]
left_toe_A_vel = digit_data[:, 48]
left_toe_B_vel = digit_data[:, 49]

# Joint velocities - Left arm (50-53)
left_shoulder_roll_vel = digit_data[:, 50]
left_shoulder_pitch_vel = digit_data[:, 51]
left_shoulder_yaw_vel = digit_data[:, 52]
left_elbow_vel = digit_data[:, 53]

# Joint velocities - Right leg (54-59)
right_hip_roll_vel = digit_data[:, 54]
right_hip_yaw_vel = digit_data[:, 55]
right_hip_pitch_vel = digit_data[:, 56]
right_knee_vel = digit_data[:, 57]
right_toe_A_vel = digit_data[:, 58]
right_toe_B_vel = digit_data[:, 59]

# Joint velocities - Right arm (60-63)
right_shoulder_roll_vel = digit_data[:, 60]
right_shoulder_pitch_vel = digit_data[:, 61]
right_shoulder_yaw_vel = digit_data[:, 62]
right_elbow_vel = digit_data[:, 63]

# Passive joint velocities - Left leg (64-68)
left_shin_vel = digit_data[:, 64]
left_tarsus_vel = digit_data[:, 65]
left_toe_pitch_vel = digit_data[:, 66]
left_toe_roll_vel = digit_data[:, 67]
left_heel_spring_vel = digit_data[:, 68]

# Passive joint velocities - Right leg (69-73)
right_shin_vel = digit_data[:, 69]
right_tarsus_vel = digit_data[:, 70]
right_toe_pitch_vel = digit_data[:, 71]
right_toe_roll_vel = digit_data[:, 72]
right_heel_spring_vel = digit_data[:, 73]

# Orientation angles (74-76)
roll = digit_data[:, 74]
pitch = digit_data[:, 75]
yaw = digit_data[:, 76]

# Height (77)
height = digit_data[:, 77]

# Reference velocities - Left leg (78-83)
LeftHipRoll_ref_vel = digit_data[:, 78]
LeftHipYaw_ref_vel = digit_data[:, 79]
LeftHipPitch_ref_vel = digit_data[:, 80]
LeftKnee_ref_vel = digit_data[:, 81]
LeftToeA_ref_vel = digit_data[:, 82]
LeftToeB_ref_vel = digit_data[:, 83]

# Reference velocities - Left arm (84-87)
LeftShoulderRoll_ref_vel = digit_data[:, 84]
LeftShoulderPitch_ref_vel = digit_data[:, 85]
LeftShoulderYaw_ref_vel = digit_data[:, 86]
LeftElbow_ref_vel = digit_data[:, 87]

# Reference velocities - Right leg (88-93)
RightHipRoll_ref_vel = digit_data[:, 88]
RightHipYaw_ref_vel = digit_data[:, 89]
RightHipPitch_ref_vel = digit_data[:, 90]
RightKnee_ref_vel = digit_data[:, 91]
RightToeA_ref_vel = digit_data[:, 92]
RightToeB_ref_vel = digit_data[:, 93]

# Reference velocities - Right arm (94-97)
RightShoulderRoll_ref_vel = digit_data[:, 94]
RightShoulderPitch_ref_vel = digit_data[:, 95]
RightShoulderYaw_ref_vel = digit_data[:, 96]
RightElbow_ref_vel = digit_data[:, 97]

# Joint torques - Left leg (98-103)
left_hip_roll_tau = digit_data[:, 98]
left_hip_yaw_tau = digit_data[:, 99]
left_hip_pitch_tau = digit_data[:, 100]
left_knee_tau = digit_data[:, 101]
left_toe_A_tau = digit_data[:, 102]
left_toe_B_tau = digit_data[:, 103]

# Joint torques - Left arm (104-107)
left_shoulder_roll_tau = digit_data[:, 104]
left_shoulder_pitch_tau = digit_data[:, 105]
left_shoulder_yaw_tau = digit_data[:, 106]
left_elbow_tau = digit_data[:, 107]

# Joint torques - Right leg (108-113)
right_hip_roll_tau = digit_data[:, 108]
right_hip_yaw_tau = digit_data[:, 109]
right_hip_pitch_tau = digit_data[:, 110]
right_knee_tau = digit_data[:, 111]
right_toe_A_tau = digit_data[:, 112]
right_toe_B_tau = digit_data[:, 113]

# Joint torques - Right arm (114-117)
right_shoulder_roll_tau = digit_data[:, 114]
right_shoulder_pitch_tau = digit_data[:, 115]
right_shoulder_yaw_tau = digit_data[:, 116]
right_elbow_tau = digit_data[:, 117]

# Joint torque commands - Left leg (118-123)
left_hip_roll_tau_cmd = digit_data[:, 118]
left_hip_yaw_tau_cmd = digit_data[:, 119]
left_hip_pitch_tau_cmd = digit_data[:, 120]
left_knee_tau_cmd = digit_data[:, 121]
left_toe_A_tau_cmd = digit_data[:, 122]
left_toe_B_tau_cmd = digit_data[:, 123]

# Joint torque commands - Left arm (124-127)
left_shoulder_roll_tau_cmd = digit_data[:, 124]
left_shoulder_pitch_tau_cmd = digit_data[:, 125]
left_shoulder_yaw_tau_cmd = digit_data[:, 126]
left_elbow_tau_cmd = digit_data[:, 127]

# Joint torque commands - Right leg (128-133)
right_hip_roll_tau_cmd = digit_data[:, 128]
right_hip_yaw_tau_cmd = digit_data[:, 129]
right_hip_pitch_tau_cmd = digit_data[:, 130]
right_knee_tau_cmd = digit_data[:, 131]
right_toe_A_tau_cmd = digit_data[:, 132]
right_toe_B_tau_cmd = digit_data[:, 133]

# Joint torque commands - Right arm (134-137)
right_shoulder_roll_tau_cmd = digit_data[:, 134]
right_shoulder_pitch_tau_cmd = digit_data[:, 135]
right_shoulder_yaw_tau_cmd = digit_data[:, 136]
right_elbow_tau_cmd = digit_data[:, 137]

# Position targets - Left leg (138-143)
left_hip_roll_pos_tgt = digit_data[:, 138]
left_hip_yaw_pos_tgt = digit_data[:, 139]
left_hip_pitch_pos_tgt = digit_data[:, 140]
left_knee_pos_tgt = digit_data[:, 141]
left_toe_A_pos_tgt = digit_data[:, 142]
left_toe_B_pos_tgt = digit_data[:, 143]

# Position targets - Right leg (144-149)
right_hip_roll_pos_tgt = digit_data[:, 144]
right_hip_yaw_pos_tgt = digit_data[:, 145]
right_hip_pitch_pos_tgt = digit_data[:, 146]
right_knee_pos_tgt = digit_data[:, 147]
right_toe_A_pos_tgt = digit_data[:, 148]
right_toe_B_pos_tgt = digit_data[:, 149]

# IMU data (150-155)
gyro_x = digit_data[:, 150]
gyro_y = digit_data[:, 151]
gyro_z = digit_data[:, 152]
acc_x = digit_data[:, 153]
acc_y = digit_data[:, 154]
acc_z = digit_data[:, 155]

# Control signals (156-157)
turning_direction = digit_data[:, 156]
cmd = digit_data[:, 157]

# Stability array (158-167)
stability_0 = digit_data[:, 158]
stability_1 = digit_data[:, 159]
stability_2 = digit_data[:, 160]
stability_3 = digit_data[:, 161]
stability_4 = digit_data[:, 162]
stability_5 = digit_data[:, 163]
stability_6 = digit_data[:, 164]
stability_7 = digit_data[:, 165]
stability_8 = digit_data[:, 166]
stability_9 = digit_data[:, 167]

# Traction and terminal (168-169)
traction = digit_data[:, 168]
terminal = digit_data[:, 169]

# Contact planning and phase (170-174)
lfoot_contact_plan = digit_data[:, 170]
lfoot_contact = digit_data[:, 171]
rfoot_contact_plan = digit_data[:, 172]
rfoot_contact = digit_data[:, 173]
phase_variable = digit_data[:, 174]

# Left foot ground reaction forces (175-180)
lfoot_grf_x = digit_data[:, 175]
lfoot_grf_y = digit_data[:, 176]
lfoot_grf_z = digit_data[:, 177]
lfoot_grt_x = digit_data[:, 178]
lfoot_grt_y = digit_data[:, 179]
lfoot_grt_z = digit_data[:, 180]

# Right foot ground reaction forces (181-186)
rfoot_grf_x = digit_data[:, 181]
rfoot_grf_y = digit_data[:, 182]
rfoot_grf_z = digit_data[:, 183]
rfoot_grt_x = digit_data[:, 184]
rfoot_grt_y = digit_data[:, 185]
rfoot_grt_z = digit_data[:, 186]

# Left foot pose (187-193)
lfoot_pos_x = digit_data[:, 187]
lfoot_pos_y = digit_data[:, 188]
lfoot_pos_z = digit_data[:, 189]
lfoot_quat_w = digit_data[:, 190]

# Check if we have all columns as indicated by header
if digit_data.shape[1] >= 198:
    lfoot_quat_i = digit_data[:, 191]
    lfoot_quat_j = digit_data[:, 192]
    lfoot_quat_k = digit_data[:, 193]
    
    # Right foot pose (194-200)
    rfoot_pos_x = digit_data[:, 194]
    rfoot_pos_y = digit_data[:, 195]
    rfoot_pos_z = digit_data[:, 196]
    rfoot_quat_w = digit_data[:, 197]
    rfoot_quat_i = digit_data[:, 198]
    rfoot_quat_j = digit_data[:, 199]
    rfoot_quat_k = digit_data[:, 200]
    print(f"Parsed all {digit_data.shape[1]} columns into named variables")
else:
    print(f"Parsed {digit_data.shape[1]} columns (stopping at column 190)")
