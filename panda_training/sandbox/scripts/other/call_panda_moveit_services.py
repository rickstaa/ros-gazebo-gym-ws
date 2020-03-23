"""Simple script to test wheter the panda_moveit_services are working\
as expceted."""

import rospy
from panda_training.srv import (
    GetEe,
    GetEeRequest,
    SetEe,
    SetEeRequest,
    SetJointPose,
    SetJointPoseRequest,
    SetEePose,
    SetEePoseRequest,
)

if __name__ == "__main__":

    # Create ROS node
    rospy.init_node("Test panda_moveit_services")

    # Connect to services
    get_ee_client = rospy.ServiceProxy(
        "/panda_moveit_planner_server/set_joint_pose", SetJointPose
    )
    req = SetJointPoseRequest()
    ee_name = get_ee_client(req)
    print(ee_name)

    # set_ee_client = rospy.ServiceProxy("panda_moveit_planner_server/set_ee", SetEe)
    # req = SetEeRequest()
    # req.ee_name = "testestestesresresrserse"
    # response = set_ee_client(req)

    # get_ee_client = rospy.ServiceProxy("panda_moveit_planner_server/get_ee", GetEe)
    # req = GetEeRequest()
    # ee_name = get_ee_client(req)
    # print(ee_name)
