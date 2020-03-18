"""Script used to test the 'panda_control_server' services"""

# %% Imports
import rospy
from panda_training.srv import (
    setJointEffortsRequest,
    setJointEfforts,
    setJointPositions,
    setJointPositionsRequest,
)

# %% /panda_control_server/set_joint_efforts test
# Connect to /panda_control_server/set_joint_efforts
rospy.logdebug(
    "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
)
rospy.wait_for_service("/panda_control_server/panda_arm/set_joint_efforts", timeout=10)
set_joint_efforts_client = rospy.ServiceProxy(
    "/panda_control_server/panda_arm/set_joint_efforts", setJointEfforts
)
rospy.logdebug(
    "Connected to 'panda_control_server/panda_arm/set_joint_efforts' service!"
)

# Generate joint_efforts msg
set_joint_efforts_msg = setJointEffortsRequest()
set_joint_efforts_msg.joint_efforts.data = [50, 50, 50, 50, 50, 50, 50]
while not rospy.is_shutdown():
    retval = set_joint_efforts_client.call(set_joint_efforts_msg)

# %% /panda_control_server/set_joint_positions test

# # Connect to /panda_control_server/set_joint_positions
# rospy.logdebug(
#     "Connecting to '/panda_control_server/panda_arm/set_joint_positions' service."
# )
# rospy.wait_for_service(
#     "/panda_control_server/panda_arm/set_joint_positions", timeout=10
# )
# set_joint_positions_client = rospy.ServiceProxy(
#     "/panda_control_server/panda_arm/set_joint_positions", setJointPositions
# )
# rospy.logdebug(
#     "Connected to 'panda_control_server/panda_arm/set_joint_positions' service!"
# )

# # Generate joint_efforts msg
# set_joint_positions_msg = setJointPositionsRequest()
# set_joint_positions_msg.joint_positions.data = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
# # set_joint_positions_msg.joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# set_joint_positions_msg.wait.data = True
# retval = set_joint_positions_client.call(set_joint_positions_msg)
# print("jan")

# # %% Send control control commands multiple times
# while not rospy.is_shutdown():
#     retval = set_joint_positions_client.call(set_joint_positions_msg)
#     retval = set_joint_efforts_client.call(set_joint_efforts_msg)
