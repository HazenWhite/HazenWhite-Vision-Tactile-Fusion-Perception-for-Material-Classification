import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_joint_angles(rtde_receive):
    """
    Get the current joint angles of the UR5 robotic arm.
    Parameters:
        rtde_receive: An instance of RTDEReceiveInterface
    Returns:
        list: A list containing the six joint angles in radians [q0, q1, q2, q3, q4, q5]
    """
    joint_angles = rtde_receive.getActualQ()
    return joint_angles


# 1. Get current TCP pose

def get_tcp_pose(rtde_receive):
    """
    Get the current TCP pose of the robot arm.
    Args:
        rtde_receive: RTDEReceiveInterface instance
    Returns:
        list: [x, y, z, rx, ry, rz]
    """
    tcp_pose = rtde_receive.getActualTCPPose()
    return tcp_pose

# Compute sensor alignment to make Z axis align with `n_base`
def compute_sensor_alignment(n_base):
    z_axis = np.array([0, 0, 1])  # Default sensor Z axis
    axis = np.cross(z_axis, n_base)
    if np.linalg.norm(axis) < 1e-6:
        return np.eye(3)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(z_axis, n_base))
    return R.from_rotvec(axis * angle).as_matrix()

# Compute alignment for X axis to match `n_base`
def compute_x_axis_alignment(n_base):
    x_axis = np.array([1, 0, 0])  # Default X axis
    axis = np.cross(x_axis, n_base)
    if np.linalg.norm(axis) < 1e-6:
        return np.eye(3)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(x_axis, n_base))
    return R.from_rotvec(axis * angle).as_matrix()


# 2. Coordinate transformation

def compute_target_tcp_pose(current_tcp_pose, P_cam, n_target):
    """
    Compute new TCP pose to align the sensor with the target point.
    """
    x, y, z, rx, ry, rz = current_tcp_pose

    T_base_tool = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    T_tool_cam = np.array([
        [0, 1, 0, -0.255],
        [1, 0, 0, 0],
        [0, 0, -1, 0.028],
        [0, 0, 0, 1]
    ])

    T_sensor_tool = np.array([
        [1, 0, 0, -0.215],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T_base_cam = np.array([
        [0, 1, 0, x - 0.255],
        [1, 0, 0, y],
        [0, 0, -1, z],
        [0, 0, 0, 1]
    ])

    P_cam_h = np.array([*P_cam, 1])
    P_base_h = np.dot(T_base_cam, P_cam_h)
    P_base = P_base_h[:3]

    n_target = np.array(n_target) / np.linalg.norm(n_target)
    n_base = np.dot(T_base_cam[:3, :3], n_target)
    n_base = n_base / np.linalg.norm(n_base)

    R_align = compute_sensor_alignment(n_base)

    T_base_sensor = np.eye(4)
    T_base_sensor[:3, :3] = R_align
    T_base_sensor[:3, 3] = P_base + np.array([0.215, 0, 0])

    target_tcp_pose = list(T_base_sensor[:3, 3]) + list(R.from_matrix(T_base_sensor[:3, :3]).as_euler('xyz'))

    return target_tcp_pose

def compute_target_tcp_pose_plus(current_tcp_pose, P_cam, n_target, x_target):
    """
    Compute new TCP pose to align the sensor with the target point and also align the X axis.
    """
    x, y, z, rx, ry, rz = current_tcp_pose

    T_base_cam = np.array([
        [0, 1, 0, x - 0.255],
        [1, 0, 0, y],
        [0, 0, -1, z],
        [0, 0, 0, 1]
    ])

    P_cam_h = np.array([*P_cam, 1])
    P_base_h = np.dot(T_base_cam, P_cam_h)
    P_base = P_base_h[:3]

    n_target = np.array(n_target) / np.linalg.norm(n_target)
    n_base = np.dot(T_base_cam[:3, :3], n_target)
    n_base = n_base / np.linalg.norm(n_base)

    x_target = np.dot(T_base_cam[:3, :3], x_target)
    x_target = x_target / np.linalg.norm(x_target)

    R_align_z = compute_sensor_alignment(n_base)
    R_align_x = compute_x_axis_alignment(x_target)

    R_final = np.dot(R_align_x, R_align_z)

    T_base_sensor = np.eye(4)
    T_base_sensor[:3, :3] = R_final
    T_base_sensor[:3, 3] = P_base + np.array([0.215, 0, 0])

    target_tcp_pose = list(T_base_sensor[:3, 3]) + list(R.from_matrix(T_base_sensor[:3, :3]).as_euler('xyz'))

    return target_tcp_pose


# 3. Send movement commands

def send_move_command(socket_connection, target_pose, velocity=0.01, acceleration=0.01):
    """
    Send URScript linear move command to robot arm.
    Args:
        socket_connection: socket connection to robot
        target_pose: [x, y, z, rx, ry, rz]
        velocity: motion velocity
        acceleration: motion acceleration
    """
    command = f"""
                movel(p[{target_pose[0]}, {target_pose[1]}, {target_pose[2]}, {target_pose[3]}, {target_pose[4]}, {target_pose[5]}], v={velocity})
                """
    try:
        socket_connection.send(command.encode('utf-8'))
        print(f"Command sent: {command}")
    except Exception as e:
        print(f"Failed to send move command: {e}")

def send_joint_command(socket_connection, joint_angles, velocity=0.5, acceleration=0.5):
    """
    Send URScript joint move command.
    Args:
        socket_connection: socket connection to robot
        joint_angles: list of 6 joint angles (radians)
        velocity: joint velocity
        acceleration: joint acceleration
    """
    if len(joint_angles) != 6:
        raise ValueError("joint_angles must contain 6 values.")

    command = f"""
                movej([{joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, {joint_angles[3]}, {joint_angles[4]}, {joint_angles[5]}], v={velocity})
                """
    try:
        socket_connection.send(command.encode('utf-8'))
        print(f"Joint command sent: {command}")
    except Exception as e:
        print(f"Failed to send joint command: {e}")


# 4. Target reach detection

def has_reached_target(rtde_receive, target_position, tolerance=0.01):
    """
    Check if robot has reached target position.
    Args:
        rtde_receive: RTDEReceiveInterface instance
        target_position: [x, y, z]
        tolerance: allowed distance in meters
    Returns:
        bool: True if reached
    """
    current_position = rtde_receive.getActualTCPPose()[:3]
    distance = np.linalg.norm(np.array(current_position) - np.array(target_position))
    return distance < tolerance


# 5. Movement state checking

def is_robot_moving(rtde_receive):
    """
    Determine whether the robot is currently moving.
    Args:
        rtde_receive: RTDEReceiveInterface instance
    Returns:
        bool: True if moving
    """
    speed_vector = rtde_receive.getActualTCPSpeed()
    speed_magnitude = np.linalg.norm(speed_vector)
    return speed_magnitude > 1e-3


# 6. Angle conversion utilities

def degrees_to_radians(degrees):
    """
    Convert list of degrees to radians.
    Args:
        degrees: list of angles in degrees
    Returns:
        list: angles in radians
    """
    return [math.radians(angle) for angle in degrees]

def radians_to_degrees(radians):
    """
    Convert list of radians to degrees.
    Args:
        radians: list of angles in radians
    Returns:
        list: angles in degrees
    """
    return [math.degrees(angle) for angle in radians]


# (Optional) 7. IK request placeholder (Note: not directly functional over socket, needs RTDE script execution)

def get_inverse_kin(socket_connection, target_pose):
    """
    Send URScript command to solve inverse kinematics (Note: result not returned via socket).
    Args:
        socket_connection: socket connection to UR5 controller
        target_pose: [x, y, z, rx, ry, rz]
    """
    command = f"""
    def ik_solver():
        pose = p[{target_pose[0]}, {target_pose[1]}, {target_pose[2]}, {target_pose[3]}, {target_pose[4]}, {target_pose[5]}]
        joint_angles = get_inverse_kin(pose)
        textmsg("IK result: ", joint_angles)
        return joint_angles
    end
    """
    try:
        socket_connection.send(command.encode('utf-8'))
        print(f"Inverse kinematics command sent: {command}")
    except Exception as e:
        print(f"Failed to send IK command: {e}")
