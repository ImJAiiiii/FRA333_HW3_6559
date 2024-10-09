# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส
1.อภิชญา_6559
'''
#import library
from HW3_utils import FKHW3
import numpy as np

#define variable
q_initial = [0, 0, 0]

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # Use FKHW3 to get the position and rotation matrices
    R, P, R_e, p_e = FKHW3(q)
    
    # Initialize Jacobian matrix
    J = np.zeros((6, 3))  # 6 rows (3 for position, 3 for orientation) and 3 columns (for 3 joints)

    # Iterate over joints to compute Jacobian columns
    for i in range(3):
        p_i = P[:, i]  # Position of joint i

        # z_i-1: The z-axis of the i-th joint frame (rotational axis)
        z_i= R[:, 2, i]

        # Translational Jacobian: Jv = z_i-1 × (p_e - p_i)
        Jv = np.cross(z_i, p_e - p_i)

        # Rotational Jacobian: Jw = z_i-1 (for revolute joints)
        Jw = z_i

        # Update the Jacobian matrix
        J[0:3, i] = Jv   # First 3 rows for translational part
        J[3:6, i] = Jw   # Last 3 rows for rotational part

    # Multiply J by the end-effector rotation matrix to get Jacobian with respect to base frame
    Je = np.dot(J[0:3, :], R_e)  # For translational part
    Jw = J[3:6, :]  # Rotational part remains the same

    # Combine both translational and rotational parts
    Je_full = np.vstack((Je, Jw))

    return Je_full # คืนค่า Jacobian ขนาด 6x3
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#
