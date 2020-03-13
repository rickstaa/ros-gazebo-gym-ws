"""Script used to test the 'panda_control_server' services"""

import rospy
from panda_training.srv import (
    setJointEffortsRequest,
    setJointEfforts,
    setJointPositions,
    setJointPositionsRequest,
)

if __name__ == "__main__":

    # #%% /panda_control_server/set_joint_efforts test
    # # Connect to /panda_control_server/set_joint_efforts
    # rospy.logdebug(
    #     "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
    # )
    # rospy.wait_for_service(
    #     "/panda_control_server/panda_arm/set_joint_efforts", timeout=10
    # )
    # set_joint_efforts_client = rospy.ServiceProxy(
    #     "/panda_control_server/panda_arm/set_joint_efforts", setJointEfforts
    # )
    # rospy.logdebug(
    #     "Connected to 'panda_control_server/panda_arm/set_joint_efforts' service!"
    # )

    # # Generate joint_efforts msg
    # set_joint_efforts_msg = setJointEffortsRequest()
    # set_joint_efforts_msg.joint_efforts.data = [0, 5, 10, 0, 0, 0, 0]

    # Send control control command
    # while not rospy.is_shutdown():
    #     retval = set_joint_efforts_client.call(set_joint_efforts_msg)
    #     print(retval)

    # /panda_control_server/set_joint_positions test
    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_arm/set_joint_positions' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_arm/set_joint_positions", timeout=10
    )
    set_joint_positions_client = rospy.ServiceProxy(
        "/panda_control_server/panda_arm/set_joint_positions", setJointPositions
    )
    rospy.logdebug(
        "Connected to 'panda_control_server/panda_arm/set_joint_positions' service!"
    )

    # Generate joint_efforts msg
    set_joint_positions_msg = setJointPositionsRequest()
    set_joint_positions_msg.joint_positions.data = [0, 1.5, 0.0, 0, 1.5, 0, 0]

    retval = set_joint_positions_client.call(set_joint_positions_msg)
    print(retval)
    # Send control control command
    # while not rospy.is_shutdown():
    #     retval = set_joint_positions_client.call(set_joint_positions_msg)
    retval = set_joint_positions_client.call(set_joint_positions_msg)
    print(retval)
