# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
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
q_initial = [0, 0, 0]
q_singulality = [0, pi/4, 3.13]
w = np.array([0, 0, 0, 10, 0, 0])

#Find MDH parameter
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
     # คำนวณ Jacobian ด้วย Robotics Toolbox
    J_toolbox = robot.jacobe(q)
    
    return J_toolbox

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def check_singularity_with_robotics_toolbox(q: list[float]):
    J_toolbox = robot.jacobe(q)
    
    J_toolbox_reduced = J_toolbox[:3, :3]

    det_J_toolbox = np.linalg.det(J_toolbox_reduced)

    print("det_J_toolbox: ", det_J_toolbox)

    if abs(det_J_toolbox) < 0.001:
        return 1  # Near singularity
    else:
        return 0  # Not near singularity
    
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def check_effort_with_robotics_toolbox(q:list[float], w:list[float])->list[float]:
    J_toolbox = robot.jacobe(q)

    tau_toolbox = np.dot(J_toolbox.T, w)

    return tau_toolbox

#==============================================================================================================#

print("------------------------check Jacobian------------------------")
J = endEffectorJacobianHW3(q_initial)
print("From my code - Jacobian: \n", J)
J_toolbox = check_results_with_robotics_toolbox(q_initial)
print("From robotics toolbox: \n", J_toolbox)
print("Differance Jacobin: \n", np.abs(J_toolbox-J))
print("Jacobian ที่เขียนเองถูกต้องหรือไม่: ", np.allclose(J_toolbox, J, atol=0.001))

print("-----------------------check Singularity-----------------------")
flag = checkSingularityHW3(q_singulality)
print("From my code - Near singularity:", flag)
flag_toolbox = check_singularity_with_robotics_toolbox(q_singulality)
print("From robotics toolbox - Near singularity:", flag_toolbox)

print("-------------------------check Effort-------------------------")
tau = computeEffortHW3(q_initial, w)
print("From my code - Efford:", tau)
tau_toolbox = check_effort_with_robotics_toolbox(q_initial, w)
print("From robotics toolbox - Efford:", tau_toolbox)

