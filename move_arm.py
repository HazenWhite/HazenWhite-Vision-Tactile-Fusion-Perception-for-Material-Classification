from cloud_move_arm import *
import socket
import time
from rtde_receive import RTDEReceiveInterface

# ======================
# Main Processing Flow
# ======================
if __name__ == "__main__":
    # Robot arm parameters
    HOST = "169.254.87.23"
    PORT = 30002

    try:
        # Connect to the robot arm
        rtde_receive = RTDEReceiveInterface(HOST)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print("Successfully connected to the robot arm.")

        # Home position of the robot arm (default joint angles in radians)
        HOME_JOINTS_RAD = [-1.576836411152975, -1.0409606138812464, -2.2127440611468714, -1.7762802282916468, -0.07978469530214483, 5.024059295654297]
        send_joint_command(s, HOME_JOINTS_RAD, velocity=0.5)
        time.sleep(5)
        print("Robot arm has successfully returned to home position.")

        current_tcp_pose = get_tcp_pose(rtde_receive)
        P_cam = [ ]         # Camera position in camera coordinates
        n_target = [ ]      # Normal vector of the target plane
        target_pose = compute_target_tcp_pose(current_tcp_pose, P_cam, n_target)
        send_move_command(s, target_pose, velocity=0.03)

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Close resources
        s.close()
        print("Program terminated.")
