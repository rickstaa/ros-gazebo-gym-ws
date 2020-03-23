import rospy
from my_fetch_robot_training.srv import (
    GetEePose,
    GetEePoseRequest,
    GetEeRpy,
    GetEeRpyRequest,
    SetEePose,
    SetEePoseRequest,
    SetJointPose,
    SetJointPoseRequest,
)

# Connect to service
joint_traj_client = rospy.ServiceProxy(
    "panda_moveit_planner_server/set_joint_pose", SetJointPose
)

# Set up a trajectory message to publish.
joint_point = SetJointPoseRequest()

joint_point.point.positions = [None] * 7
joint_point.point.positions[0] = 0.0
joint_point.point.positions[1] = 0.0
joint_point.point.positions[2] = 0.0
joint_point.point.positions[3] = -1.5
joint_point.point.positions[4] = 0.0
joint_point.point.positions[5] = 1.5
joint_point.point.positions[6] = 0.0

result = joint_traj_client(joint_point)
