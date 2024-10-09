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
import roboticstoolbox as rtb
import matplotlib
matplotlib.use('TkAgg')  # บังคับใช้ Tkinter backend
import matplotlib.pyplot as plt

from HW3_utils import FKHW3
from FRA333_HW3_6559 import endEffectorJacobianHW3
from spatialmath import SE3

#define variable
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = [0, 0, 0]

#Find DH parameter
L1 = rtb.RevoluteDH(a=0, alpha=0, d=d_1, offset=np.pi)  # ข้อต่อที่ 1
L2 = rtb.RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)  # ข้อต่อที่ 2
L3 = rtb.RevoluteDH(a=-a_2, alpha=0,  d=0, offset=0)  # ข้อต่อที่ 3

robot = rtb.DHRobot([L1, L2, L3], 
                    tool=SE3.Rt(np.array([[0, 0, -1],
                                          [0, 1, 0],
                                          [1, 0, 0]]),
                                np.array([-(a_3+d_6), -d_5, d_4]))
                    , name='3DOFRobot')


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
# ฟังก์ชันตรวจสอบ Jacobian ด้วย Robotics Toolbox
def check_results_with_robotics_toolbox(q: list[float]):
     # คำนวณ Jacobian ด้วย Robotics Toolbox
    J_toolbox = robot.jacobe(q)
    
    # คำนวณ Jacobian ด้วยฟังก์ชันที่คุณเขียนเอง
    J_custom = endEffectorJacobianHW3(q)
    
    # เปรียบเทียบ Jacobian
    print("Jacobian จาก Robotics Toolbox:\n", J_toolbox)
    print("Jacobian จากโค้ดที่เขียน:\n", J_custom)
    difference_jacobian = np.abs(J_toolbox - J_custom)
    print("ความแตกต่างระหว่าง Jacobian:\n", difference_jacobian)
    print(np.allclose(J_toolbox, J_custom, atol=1e-6))

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#
q = q_initial  # ค่าข้อต่อสำหรับการทดสอบ
# print(robot)
# robot.plot([0, 0, 0])
# plt.show()
# input("Press Enter to exit...")  # รอ input ก่อนปิดโปรแกรม

check_results_with_robotics_toolbox(q)

