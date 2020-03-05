"""Simple script to test wheter the panda_moveit_services are working\
as expceted."""

import rospy
from panda_reach_training.srv import (
    getEePose,
    getEePoseRequest,
    getEeRpy,
    getEeRpyRequest,
)

if __name__ == "__main__":

    # Create ROS node
    rospy.init_node("Test panda_moveit_services")

    # Connect to services
    # ee_pose_client = rospy.ServiceProxy(
    #     "panda_moveit_planner_server/get_ee_pose", getEePose
    # )
    # gripper_pose_req = getEePoseRequest()
    # gripper_pose = ee_pose_client(gripper_pose_req)
    ee_rpy_client = rospy.ServiceProxy(
        "panda_moveit_planner_server/get_ee_rpy", getEeRpy
    )
    gripper_rpy_req = getEeRpyRequest()
    gripper_rpy = ee_rpy_client(gripper_rpy_req)
