"""Simple script to test wheter the panda_moveit_services are working\
as expceted."""

import rospy
from panda_training.srv import (
    getEe,
    getEeRequest,
    setEe,
    setEeRequest,
    setJointPose,
    setJointPoseRequest,
    setEePose,
    setEePoseRequest,
)

if __name__ == "__main__":

    # Create ROS node
    rospy.init_node("Test panda_moveit_services")

    # Connect to services
    get_ee_client = rospy.ServiceProxy(
        "/panda_moveit_planner_server/set_ee_pose", setEePose
    )
    req = setEePoseRequest()
    ee_name = get_ee_client(req)
    print(ee_name)

    # set_ee_client = rospy.ServiceProxy("panda_moveit_planner_server/set_ee", setEe)
    # req = setEeRequest()
    # req.ee_name = "testestestesresresrserse"
    # response = set_ee_client(req)

    # get_ee_client = rospy.ServiceProxy("panda_moveit_planner_server/get_ee", getEe)
    # req = getEeRequest()
    # ee_name = get_ee_client(req)
    # print(ee_name)
