"""Script used to test the 'panda_control_server' services"""

# Imports
import rospy
from panda_training.srv import (
    setJointEffortsRequest,
    setJointEfforts,
    setJointPositions,
    setJointPositionsRequest,
)

if __name__ == "__main__":

    # # Initiate ROS nodetypes
    # rospy.init_node("panda_control_server_test")

    # ######## - TEST SET ARM JOINT EFFORTS - #########
    # # %% /panda_control_server/panda_arm/set_joint_efforts test
    # # Connect to /panda_control_server/set_joint_efforts
    # rospy.logdebug(
    #     "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
    # )
    # rospy.wait_for_service(
    #     "/panda_control_server/panda_arm/set_joint_efforts", timeout=10
    # )
    # set_arm_joint_positions_msg = rospy.ServiceProxy(
    #     "/panda_control_server/panda_arm/set_joint_efforts", setJointEfforts
    # )
    # rospy.logdebug(
    #     "Connected to 'panda_control_server/panda_arm/set_joint_efforts' service!"
    # )

    # # Generate joint_efforts msg
    # set_arm_joint_efforts_msg = setJointEffortsRequest()
    # set_arm_joint_efforts_msg.joint_names = ["panda_joint1", "test"]
    # # set_arm_joint_efforts_msg.joint_efforts.data = [0, 0, 0, 0, 0, 0, 0]
    # set_arm_joint_efforts_msg.joint_efforts.data = [0, 0]
    # retval = set_arm_joint_positions_msg.call(set_arm_joint_efforts_msg)
    # print("done")

    # ######### - TEST SET ARM JOINT POSITIONS - #########
    # # #%% /panda_control_server/panda_arm/set_joint_positions test

    # # Connect to /panda_control_server/set_joint_positions
    # rospy.logdebug(
    #     "Connecting to '/panda_control_server/panda_arm/set_joint_positions' service."
    # )
    # rospy.wait_for_service(
    #     "/panda_control_server/panda_arm/set_joint_positions", timeout=10
    # )
    # set_arm_joint_positions_msg = rospy.ServiceProxy(
    #     "/panda_control_server/panda_arm/set_joint_positions", setJointPositions
    # )
    # rospy.logdebug(
    #     "Connected to 'panda_control_server/panda_arm/set_joint_positions' service!"
    # )

    # # Generate joint_efforts msg
    # set_arm_joint_positions_msg = setJointPositionsRequest()
    # # set_arm_joint_positions_msg.joint_names = ["panda_joint1", "panda_joint6"]
    # # set_arm_joint_positions_msg.joint_positions.data = [1.5, 2]
    # # set_arm_joint_positions_msg.joint_positions.data = [0.0, 0.0, 0.0, 1.5, 1.5, 0.0, 0.0]
    # set_arm_joint_positions_msg.joint_positions.data = [
    #     1.5,
    #     1.0,
    #     1.0,
    #     1.5,
    #     1.5,
    #     1.0,
    #     1.0,
    # ]
    # # set_arm_joint_positions_msg.joint_positions.data = [0.0, 1.5]
    # set_arm_joint_positions_msg.wait.data = True
    # retval = set_arm_joint_positions_msg.call(set_arm_joint_positions_msg)
    # print("done")

    # ######### - TEST SET HAND EFFORTS - #########
    # # %% /panda_control_server/panda_hand/set_joint_positions test
    # # Connect to /panda_control_server/set_joint_positions
    # rospy.logdebug(
    #     "Connecting to '/panda_control_server/panda_hand/set_joint_efforts' service."
    # )
    # rospy.wait_for_service(
    #     "/panda_control_server/panda_hand/set_joint_efforts", timeout=10
    # )
    # set_arm_joint_efforts_msg = rospy.ServiceProxy(
    #     "/panda_control_server/panda_hand/set_joint_efforts", setJointEfforts
    # )
    # rospy.logdebug(
    #     "Connected to 'panda_control_server/panda_hand/set_joint_efforts' service!"
    # )

    # # Generate joint_efforts msg
    # set_hand_joint_efforts_msg = setJointEffortsRequest()
    # set_hand_joint_efforts_msg.joint_names = [
    #     "panda_finger_joint1",
    #     "panda_finger_joint2",
    # ]
    # set_hand_joint_efforts_msg.joint_efforts.data = [-0.08, -0.08]
    # set_hand_joint_efforts_msg.wait.data = True
    # retval = set_arm_joint_efforts_msg.call(set_hand_joint_efforts_msg)
    # print("done")

    # ######### - TEST SET HAND JOINT POSITIONS - #########
    # # %% /panda_control_server/panda_hand/set_joint_positions test
    # # Connect to /panda_control_server/set_joint_positions
    # rospy.logdebug(
    #     "Connecting to '/panda_control_server/panda_hand/set_joint_positions' service."
    # )
    # rospy.wait_for_service(
    #     "/panda_control_server/panda_hand/set_joint_positions", timeout=10
    # )
    # set_hand_joint_positions_srv = rospy.ServiceProxy(
    #     "/panda_control_server/panda_hand/set_joint_positions", setJointPositions
    # )
    # rospy.logdebug(
    #     "Connected to 'panda_control_server/panda_hand/set_joint_positions' service!"
    # )

    # # Generate joint_efforts msg
    # set_hand_joint_positions_msg = setJointPositionsRequest()
    # set_hand_joint_positions_msg.joint_names = [
    #     "panda_finger_joint1",
    #     "panda_finger_joint2",
    # ]
    # set_hand_joint_positions_msg.joint_positions.data = [0.04, 0.04]
    # set_hand_joint_positions_msg.wait.data = True
    # retval = set_hand_joint_positions_srv.call(set_hand_joint_positions_msg)
    # print("done")
