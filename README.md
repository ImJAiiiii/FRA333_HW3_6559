# FRA333_HW3_6559
 การบ้านนี้ถูกออกแบบึ้นมาเพื่อให้ผู้เรียนได้ประยุกต์ใช้องค์ความรู้การหาจลนศาสตร์เชิงอนุพันธ์ (Differential kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator)
## Installation
### Setup Environment
To get started with the project, you'll need to set up your environment with the necessary dependencies.
#### Prerequisites
Make sure you have the following installed:
- **Python 3.x**: The base programming language.
- **`numpy`**: A library for matrix and numerical operations.
- **`Robotics Toolbox for Python`**: A toolbox for working with robot models, calculating kinematics, and analyzing dynamics.
- **`math`**: For mathematical functions, especially trigonometric calculations.
- **`spatialmath`**: A library for handling spatial transformations and manipulating 3D vectors and matrices.
- **`matplotlib`**: A library for plotting.
#### Step

1. **Clone the repository**

   Download the project to your local machine using Git:
   ```bash
   git clone https://github.com/ImJAiiiii/FRA333_HW3_6559.git
   cd robot-jacobian
2. **Create a Python virtual environment**

    To keep dependencies isolated, create and activate a virtual environment
    ```bash
    python3 -m venv env
    source env/bin/activate  # On Windows, use `env\Scripts\activate`
3. **Install the required libraries**

    Once your virtual environment is active, install all required dependencies:
    ```bash
    pip install numpy roboticstoolbox-python spatialmath-python matplotlib
## Function

### 1. Jacobian Matrix Function
**Jacobian** is Matrix in robotics which provides the relation between joint velocities $\dot{X}$  & end-effector velocities $\dot{q}$ of a robot manipulator.

$$\dot{X} = J(q) \cdot \dot{q}$$

Where:
* $\dot{X}$ is represents the velocity of the end-effector.
* $J_q$ is the Jacobian matrix that maps joint velocities 
$\dot{q}$ to end-effector velocities.

The Jacobian matrix can be expressed in expanded form as:
$$
\begin{bmatrix}
    \dot{x} \\
    \dot{y} \\
    \dot{z} \\
    \dot{\alpha} \\
    \dot{\beta} \\
    \dot{\gamma}
\end{bmatrix}_{6 \times 1}
=
\begin{bmatrix}
    J_{11} & J_{12} & \cdots & J_{1n} \\
    J_{21} & J_{22} & \cdots & J_{2n} \\
    \vdots & \vdots & \ddots & \vdots \\
    J_{61} & J_{62} & \cdots & J_{6n}
\end{bmatrix}_{6 \times n}
\cdot
\begin{bmatrix}
    \dot{q}_1 \\
    \dot{q}_2 \\
    \dot{q}_3 \\
    \vdots \\
    \dot{q}_n
\end{bmatrix}_{n \times 1}
$$


In this context:
- The **first three rows** represent the **Linear Velocity Jacobian (Jv)**, which describes how the joint velocities affect the linear velocity of the end-effector.

- The **last three rows** represent the **Angular Velocity Jacobian (Jw)**, which describes how the joint velocities affect the angular velocity of the end-effector.

$$
J = \begin{bmatrix}
    J_v \\
    J_\omega
\end{bmatrix}_{6 \times n}
$$

Where:

* $J_v$ is the part of the Jacobian that relates joint velocities to linear velocities.
* $J_ ω$ is the part of the Jacobian that relates joint velocities to angular velocities.

**Finding $J_v$ of 3R Robot**

The **Translational Jacobian (Jv)** describes how the velocities of the joints affect the linear velocity of the robot’s end-effector. To compute \(Jv\) for a 3R robot:

1. **Revolute Joint Contribution**:
   For each revolute joint $i$, the contribution to the end-effector's linear velocity is calculated using the cross product of the joint's rotation axis $z_i$ and the vector from the joint to the end-effector position:

$$ J_v = z_i \times (p_e - p_i) $$

Where:
   - $z_i$ is the axis of rotation for the $i$-th joint (often represented as the z-axis of the joint's frame).
   - $p_e$ is the position vector of the end-effector.
   - $p_i$ is the position vector of the $i$-th joint.

2. **Jv for Multiple Joints**:
    For a 3R robot, we will compute $Jv_1$, $Jv_2$, and $Jv_3$, corresponding to the contributions of the first, second, and third joints, respectively, to the linear velocity of the end-effector.

3. **Matrix Form**:
   The **Translational Jacobian** for the 3R robot is formed by stacking the contributions $Jv_1$, $Jv_2$, and $Jv_3$ together:

    $$
    J_v = \begin{bmatrix}
    z_1 \times (p_e - p_1) & z_2 \times (p_e - p_2) & z_3 \times (p_e - p_3)
    \end{bmatrix}
    $$
   
**Finding $J_\omega$ of 3R Robot**

The **Angular Velocity Jacobian (Jw)** describes the relationship between the joint angular velocities and the angular velocity of the robot’s end-effector.

$$J_\omega = \begin{bmatrix}z_1 & z_2 & z_3\end{bmatrix}$$

$J_e$

**Finding $z_i$ and $p_i$**

from `HW3_utils.py` function `FKHW3(q)` it return `R`, `P`, `R_e` and `P_e`

example: $q$ = [0, 0, 0]

`R` is 3x3x4 represent the rotation matrix of different joint
$$
R = 
        \begin{bmatrix}
            -1.0000000e+00 & -1.0000000e+00 & -1.0000000e+00 & 6.1232343e-17\\
            -1.2246469e-16 & -7.4987988e-33 & -7.4987988e-33 & 6.1232343e-17\\
            0.0000000e+00 & 1.2246469e-16 & 1.2246469e-16 & 1.0000000e+00\\
        \end{bmatrix}
$$
$$
        \begin{bmatrix}
            1.2246469e-16 & 1.2246469e-16 & 1.2246469e-16 & 1.0000000e+00\\
            -1.0000000e+00 & -6.1232343e-17 & -6.1232343e-17 & -1.2246469e-16\\
            0.0000000e+00 & 1.0000000e+00 & 1.0000000e+00 & -6.1232343e-17\\
        \end{bmatrix}
$$

$$
        \begin{bmatrix}
            0.0000000e+00 & 0.0000000e+00 & 0.0000000e+00 & 1.2246469e-16\\
            0.0000000e+00 & 1.0000000e+00 & 1.0000000e+00 & 1.0000000e+00\\
            1.0000000e+00 & 6.1232343e-17 & 6.1232343e-17 & -6.1232343e-17\\
        \end{bmatrix}
    
$$
1st Matrix (Rotation for Joint 1):
    $$
    R_1 = \begin{bmatrix}
    -1.0000000e+00 & -1.0000000e+00 & -1.0000000e+00 \\
    -1.2246469e-16 & -7.4987988e-33 & -7.4987988e-33 \\
    0.0000000e+00  & 1.2246469e-16  & 1.2246469e-16  \\
    \end{bmatrix}
    $$

2nd Matrix (Rotation for Joint 2):
    $$
    R_2 = \begin{bmatrix}
    1.2246469e-16 & 1.2246469e-16 & 1.2246469e-16 \\
    -1.0000000e+00 & -6.1232343e-17 & -6.1232343e-17 \\
    0.0000000e+00  & 1.0000000e+00  & 1.0000000e+00 \\
    \end{bmatrix}
    $$

3rd Matrix (Rotation for Joint 3):
    $$
    R_3 = \begin{bmatrix}
    0.0000000e+00 & 0.0000000e+00 & 0.0000000e+00 \\
    0.0000000e+00 & 1.0000000e+00 & 1.0000000e+00 \\
    1.0000000e+00  & 6.1232343e-17  & 6.1232343e-17 \\
    \end{bmatrix}
    $$

4th Matrix (End-effector transformation or `R_e`):
    $$
    R_e = \begin{bmatrix}
    6.1232343e-17 & 6.1232343e-17 & 1.0000000e+00 \\
    1.0000000e+00 & -1.2246469e-16 & -6.1232343e-17 \\
    1.2246469e-16  & 1.0000000e+00  & -6.1232343e-17 \\
    \end{bmatrix}
    $$
`z_i` is $R[:, 2, i]$
$$
z_1 = \begin{bmatrix}

\end{bmatrix}
$$

`P` is a 3x4 matrix represent the position vectors of different joint

$$
P = \begin{bmatrix}
    0.0000000e+00 & 0.0000000e+00 & 4.2500000e-01 & 8.9943000e-01\\
    0.0000000e+00 & 0.0000000e+00 &-5.2047489e-17 & 1.0900000e-01\\
    8.9200000e-02 & 8.9200000e-02 & 8.9200000e-02 &-3.8000000e-03\\
    \end{bmatrix}
$$
1st column represents the position of the first point: 
$$
P_1 =   \begin{bmatrix}
        0.0000000e+00 & 0.0000000e+00 & 8.9200000e-02
        \end{bmatrix}
$$
2nd column represents the position of the second point: 
$$
P_2 =   \begin{bmatrix}
        0.0000000e+00 & 0.0000000e+00 & 8.9200000e-02
        \end{bmatrix}
$$
3rd column represents the position of the third point:
$$
P_3 =   \begin{bmatrix}
        4.2500000e-01 & -5.2047489e-17 & 8.9200000e-02
        \end{bmatrix}
$$
4th column represents the position of the end-effecto or `P_e`: 
$$
P_e =   \begin{bmatrix}
        8.9943000e-01 & 1.0900000e-01 & -3.8000000e-03
        \end{bmatrix}
$$

**Check Jacobian Matrix by Robotics toolbox**
1. Define MDH Parameter

    |   a   | alpha |   d   | theta |
    |:-----:|:-----:|:-----:|:-----:|
    |   0   |   0   |  d_1  |  pi   |
    |   0   | pi/2  |   0   |   0   |
    |   0   | -a_2  |   0   |   0   |

    and add spatial transformations matrix:
    $$
    SE3 = \begin{bmatrix}
    0 & 0 & -1 & -(a_3 + d_6) \\
    0 & 1 & 0 & -d_5 \\
    1 & 0 & 0 & d_4 \\
    \end{bmatrix} 
    $$

2.   
    


    