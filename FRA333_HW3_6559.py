# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส
1.อภิชญา_6559
'''
#import library
from HW3_utils import FKHW3
import numpy as np

#=============================================<คำตอบข้อ 1>======================================================#
def endEffectorJacobianHW3(q: list[float]) -> np.ndarray:
    """
    Calculate the Jacobian matrix of the robot’s end-effector.
    
    Input:
        q (list[float]): Joint angles (3 values for 3R robot).
    
    Output:
        J_e (np.ndarray): 6x3 Jacobian matrix for the end-effector.
    """

    # Use FKHW3 to get the position and rotation matrices
    R, P, R_e, p_e = FKHW3(q)  # Forward Kinematics

    n = len(q)  # Number of joints

    # Initialize a 6xn Jacobian matrix (3 for translation, 3 for rotation)
    J = np.zeros((6, n))
    
    # Iterate over each joint to compute the columns
    for i in range(n):
        p_i = P[:, i]  # Position of joint i in the base frame
        
        # z_i: The z-axis of the i-th joint frame (rotational axis)
        z_i = R[:, 2, i]  # Extract z-axis of joint i's frame (third column of rotation matrix)
        
        # Translational Jacobian: Jv = z_i × (p_e - p_i)
        J[:3, i] = np.cross(z_i, (p_e - p_i))  # Cross product for linear velocity contribution
        
        # Rotational Jacobian: Jw = z_i (for revolute joints)
        J[3:, i] = z_i  # Rotational velocity contribution for revolute joints

    # Now, transform the full Jacobian from the base frame to the end-effector frame
    # Apply the rotation matrix R_e to both the translational and rotational parts
    J_e_translational = np.dot(R_e.T, J[:3, :])  # Transform the translational part
    J_e_rotational = np.dot(R_e.T, J[3:, :])    # Transform the rotational part

    # Combine both transformed parts
    J_e = np.vstack((J_e_translational, J_e_rotational))

    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    """
    Check if the given configuration is near a singularity.
    
    Input:
        q (list[float]): Joint angles (3 values for 3R robot).
    
    Output:
        bool: 1 if near singularity, 0 otherwise.
    """

    # Compute the full Jacobian using your custom function
    J = endEffectorJacobianHW3(q)
    
    # Extract the reduced Jacobian (the first 3 rows of the Jacobian, corresponding to position)
    J_reduced = J[:3, :3]  # Translational Jacobian part (for p_x, p_y, p_z)
    
    # Compute the determinant of the reduced Jacobian
    det_J = np.linalg.det(J_reduced)

    # Check if the determinant is less than the threshold epsilon
    if (abs(det_J) < 0.001):
        return 1  # Near singularity
    else:
        return 0  # Not near singularity

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    """
    Compute the joint efforts required to achieve a given wrench at the end-effector.
    
    Input:
        q (list[float]): Joint angles (3 values for 3R robot).
        w (list[float]): Wrench vector at the end-effector (6 values: forces and torques).
    
    Output:
        tau (list[float]): Joint torques/efforts (3 values).
    """
    
    # Compute the full Jacobian using your custom function
    J = endEffectorJacobianHW3(q)
    
    # Compute the joint torques using the transpose of the Jacobian and the wrench
    tau = np.dot(J.T, w)
    
    return tau
#==============================================================================================================#