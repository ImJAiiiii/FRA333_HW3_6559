# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส
1.อภิชญา_6559
'''
#import library
import numpy as np
from math import pi
import roboticstoolbox as rtb

import matplotlib
matplotlib.use('TkAgg')  # บังคับใช้ Tkinter backend
import matplotlib.pyplot as plt

from HW3_utils import FKHW3
from FRA333_HW3_6559 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
from spatialmath import SE3

#define variable
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

q_initial = [0, 0, 0] # Default joint angles
q_singulality = [0.0, 0.0, 3.0] # Configuration with potential singularity
#w = np.array([0, 0, 0, 10, 0, 0])

# ================== Define Robot using MDH =====================
# Create the 3R robot using the Denavit-Hartenberg parameters
robot = rtb.DHRobot([
                rtb.RevoluteMDH(alpha=0, a=0, d=d_1, offset=pi), 
                rtb.RevoluteMDH(alpha=pi/2, a=0, d=0, offset=0),
                rtb.RevoluteMDH(alpha=0, a=-a_2,  d=0, offset=0)], 
                    tool = SE3([
                    [0, 0, -1, -(a_3 + d_6)],
                    [0, 1, 0, -d_5],
                    [1, 0, 0, d_4],
                    [0, 0, 0, 1]]),
                    name = "3DOF_Robot")

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
# ฟังก์ชันตรวจสอบ Jacobian ด้วย Robotics Toolbox
def check_results_with_robotics_toolbox(q: list[float]):
    """
    Calculate the Jacobian matrix using Robotics Toolbox.
    Input: Joint angles q (list of floats)
    Output: Jacobian matrix from Robotics Toolbox (numpy array)
    """
     # คำนวณ Jacobian ด้วย Robotics Toolbox
    J_toolbox = robot.jacobe(q)
    
    return J_toolbox

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
#Find all singular configurations by iterating through joint angles.
def FindSingularity(step_size):
    """
    Check if the given configuration is near a singularity.
    Input: Joint angles q (list of floats)
    Output: 1 if near singularity, 0 otherwise
    """

    q_ranges = [np.arange(0, 2 * pi, step_size) for _ in range(3)]
    singular_configurations = []  # Store singular configurations

    # Iterate over all possible combinations of joint angles
    for q1 in q_ranges[0]:
        for q2 in q_ranges[1]:
            for q3 in q_ranges[2]:
                q = [q1, q2, q3]  # Current joint configuration
                if check_singularity_with_robotics_toolbox(q):
                    singular_configurations.append(q)

    return singular_configurations

def check_singularity_with_robotics_toolbox(q: list[float]):
    """
    Check if the given configuration is near a singularity.
    Input: Joint angles q (list of floats)
    Output: 1 if near singularity, 0 otherwise
    """

    # Compute the Jacobian matrix using the Robotics Toolbox
    J_toolbox = robot.jacobe(q)

    # Get the translational part of the Jacobian
    J_toolbox_reduced = J_toolbox[:3, :3]

    # Compute the determinant of the Jacobian
    det_J_toolbox = np.linalg.det(J_toolbox_reduced)

    if abs(det_J_toolbox) < 0.001:
        return 1  # Near singularity
    else:
        return 0  # Not near singularity
    
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def check_effort_with_robotics_toolbox(q:list[float], w:list[float])->list[float]:
    """
    Calculate the joint efforts required for a given wrench using Robotics Toolbox.
    Input: Joint angles q, Wrench w (forces and torques)
    Output: Joint efforts (torques) required
    """

    # Compute the Jacobian matrix using the Robotics Toolbox
    J_toolbox = robot.jacobe(q)

    # Calculate the joint efforts using the transpose of the Jacobian and wrench
    tau_toolbox = np.dot(J_toolbox.T, w)

    return tau_toolbox

#==============================================================================================================#
# singularity = FindSingularity(step_size=1)
# print("Singularity:",singularity)
#Output : [[0.0, 0.0, 3.0], [0.0, 1.0, 3.0], [0.0, 2.0, 3.0], [0.0, 3.0, 3.0], [0.0, 4.0, 3.0], [0.0, 5.0, 3.0], [0.0, 6.0, 3.0], [1.0, 0.0, 3.0], [1.0, 1.0, 3.0], [1.0, 2.0, 3.0], 
# [1.0, 3.0, 3.0], [1.0, 4.0, 3.0], [1.0, 5.0, 3.0], [1.0, 6.0, 3.0], [2.0, 0.0, 3.0], [2.0, 1.0, 3.0], [2.0, 2.0, 3.0], [2.0, 3.0, 3.0], [2.0, 4.0, 3.0], [2.0, 5.0, 3.0], [2.0, 6.0, 3.0], 
# [3.0, 0.0, 3.0], [3.0, 1.0, 3.0], [3.0, 2.0, 3.0], [3.0, 3.0, 3.0], [3.0, 4.0, 3.0], [3.0, 5.0, 3.0], [3.0, 6.0, 3.0], [4.0, 0.0, 3.0], [4.0, 1.0, 3.0], [4.0, 2.0, 3.0], [4.0, 3.0, 3.0], 
# [4.0, 4.0, 3.0], [4.0, 5.0, 3.0], [4.0, 6.0, 3.0], [5.0, 0.0, 3.0], [5.0, 1.0, 3.0], [5.0, 2.0, 3.0], [5.0, 3.0, 3.0], [5.0, 4.0, 3.0], [5.0, 5.0, 3.0], [5.0, 6.0, 3.0], [6.0, 0.0, 3.0], 
# [6.0, 1.0, 3.0], [6.0, 2.0, 3.0], [6.0, 3.0, 3.0], [6.0, 4.0, 3.0], [6.0, 5.0, 3.0], [6.0, 6.0, 3.0]]
# ======================= Test Functions ==========================
print("------------------------check Jacobian------------------------")
J = endEffectorJacobianHW3(q_initial) # Calculate Jacobian manually
print("From my code - Jacobian: \n", J)

J_toolbox = check_results_with_robotics_toolbox(q_initial) # Toolbox result
print("From robotics toolbox: \n", J_toolbox)

# Compare the results
print("Differance Jacobin: \n", np.abs(J_toolbox-J))
print("Jacobian ที่เขียนเองถูกต้องหรือไม่: ", np.allclose(J_toolbox, J, atol=0.001))

print("-----------------------check Singularity-----------------------")
flag = checkSingularityHW3(q_singulality) # Manual check
print("From my code - Is the configuration singular?:", flag)

flag_toolbox = check_singularity_with_robotics_toolbox(q_singulality)# Toolbox check
print("From robotics toolbox - Is the configuration singular?:", flag_toolbox)

print("-------------------------check Effort-------------------------")
outputFT300 = [10, 0, 0, 0, 0, 0] # Example wrench from FT300 sensor
w = np.array([
    outputFT300[3],  # f_x
    outputFT300[4],  # f_y
    outputFT300[5],  # f_z
    outputFT300[0],  # tau_x
    outputFT300[1],  # tau_y
    outputFT300[2]   # tau_z
]).reshape(6, 1)

tau = computeEffortHW3(q_initial, w) # Manual effort calculation
print("From my code - tau =", tau)

tau_toolbox = check_effort_with_robotics_toolbox(q_initial, w) # Toolbox effort calculation
print("From robotics toolbox - tau =", tau_toolbox)

# Compare efforts
print("Differance tau:", abs(tau_toolbox-tau))

# ======================== End of Code ==============================
