"""This script can be used to test the 'panda_moveit_server' servies"""

# Import ROS related python packages
import rospy

# Import ROS msgs and srvs
from panda_training.srv import (
    SetJointPositions,
    SetJointPositionsRequest,
    SetEePose,
    SetEePoseRequest,
)

# Main function
if __name__ == "__main__":

    # Initiate ROS node
    rospy.init_node("test_panda_moveit_server")

    # -- Test set robot joint positions service --
    req = SetJointPositionsRequest()
    # req.joint_positions.data = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 , 1.5]
    req.joint_positions.data = [0.00]
    req.joint_names = ["panda_finger_joint2"]
    set_joint_positions_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/set_joint_positions", SetJointPositions
    )
    resp = set_joint_positions_srv.call(req)
    print(resp)

    # -- Test set robot joint positions service --
    req = SetJointPositionsRequest()
    # req.joint_positions.data = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 , 1.5]
    req.joint_positions.data = [0.5]
    req.joint_names = ["panda_joint1"]
    setarm__joint_positions_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_arm/set_joint_positions", SetJointPositions
    )
    resp = setarm__joint_positions_srv.call(req)
    print(resp)

    # -- Test set robot joint positions service --
    req = SetJointPositionsRequest()
    # req.joint_positions.data = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 , 1.5]
    req.joint_positions.data = [0.03]
    req.joint_names = ["panda_finger_joint1"]
    set_hand_joint_positions_srv = rospy.ServiceProxy(
        "panda_moveit_planner_server/panda_hand/set_joint_positions", SetJointPositions
    )
    resp = set_hand_joint_positions_srv.call(req)
    print(resp)

    # -- Test set ee pose service --
    # req = SetEePoseRequest()
    # req.pose.position.x = 5
    # set_ee_pose_srv = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/panda_arm/set_ee_pos", SetEePose
    # )
    # resp = set_ee_pose_srv.call(req)
    # print(resp)
