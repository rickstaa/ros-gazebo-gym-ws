import rospy
from my_fetch_robot_training.srv import (
    EePose,
    EePoseRequest,
    EeRpy,
    EeRpyRequest,
    EeTraj,
    EeTrajRequest,
    JointTraj,
    JointTrajRequest,
)

# Connect to service
joint_traj_client = rospy.ServiceProxy("/joint_traj_srv", JointTraj)

# Set up a trajectory message to publish.
joint_point = JointTrajRequest()

joint_point.point.positions = [None] * 7
joint_point.point.positions[0] = 0.0
joint_point.point.positions[1] = 0.0
joint_point.point.positions[2] = 0.0
joint_point.point.positions[3] = -1.5
joint_point.point.positions[4] = 0.0
joint_point.point.positions[5] = 1.5
joint_point.point.positions[6] = 0.0

result = joint_traj_client(joint_point)
