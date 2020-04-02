"""Script used to test the 'panda_control_server' services"""

# Imports
import rospy
from panda_training.srv import (
    SetJointEffortsRequest,
    SetJointEfforts,
    SetJointPositions,
    SetJointPositionsRequest,
    ListControlType,
    ListControlTypeRequest,
    SwitchControlType,
    SwitchControlTypeRequest,
)

# --TESTS--
# 1. Empty message ->
# 2. Only control commands -> Success
# 3. To little or to much control commands -> Fail
# 4. Wrong joint_names --> Fail
# 5. Commands not equal to joint_names field --> fail
# 6. Equal to each other --> succes

if __name__ == "__main__":

    # Initiate ROS nodetypes
    rospy.init_node("panda_control_server_test")

    # # ######## - TEST LIST CONTROLLER TYPE SERVICE - #########
    # # %% /panda_control_server/list_control_type test
    # # Connect to /panda_control_server/list_control_type
    # rospy.logdebug("Connecting to '/panda_control_server/list_control_type' service.")
    # rospy.wait_for_service("/panda_control_server/list_control_type", timeout=10)
    # list_control_type_srv = rospy.ServiceProxy(
    #     "/panda_control_server/list_control_type", ListControlType
    # )
    # rospy.logdebug("Connected to 'panda_control_server/list_control_type' service!")

    # # Check control type
    # list_control_type_msg = ListControlTypeRequest()
    # resp = list_control_type_srv.call(list_control_type_msg)
    # print(resp.control_type)

    # # ######## - TEST switch CONTROLLER TYPE SERVICE - #########
    # # %% /panda_control_server/list_control_type test
    # # Connect to /panda_control_server/list_control_type
    # rospy.logdebug("Connecting to '/panda_control_server/switch_control_type' service.")
    # rospy.wait_for_service("/panda_control_server/switch_control_type", timeout=10)
    # switch_control_type_srv = rospy.ServiceProxy(
    #     "/panda_control_server/switch_control_type", SwitchControlType
    # )
    # rospy.logdebug("Connected to 'panda_control_server/switch_control_type' service!")

    # # Switch Control type
    # switch_control_type_msg = SwitchControlTypeRequest()
    # resp = switch_control_type_srv.call(switch_control_type_msg)
    # print(resp.success)
    # print("done")

    # # List control type
    # list_control_type_msg = ListControlTypeRequest()
    # resp = list_control_type_srv.call(list_control_type_msg)
    # print(resp.control_type)

    # # ####### - TEST SET JOINT EFFORTS - #########
    # %% /panda_control_server/set_joint_efforts test
    # Connect to /panda_control_server/set_joint_efforts
    rospy.logdebug("Connecting to '/panda_control_server/set_joint_efforts' service.")
    rospy.wait_for_service("/panda_control_server/set_joint_efforts", timeout=10)
    set_joint_effort_srv = rospy.ServiceProxy(
        "/panda_control_server/set_joint_efforts", SetJointEfforts
    )
    rospy.logdebug("Connected to 'panda_control_server/set_joint_efforts' service!")

    # Generate joint_efforts msg
    set_joint_efforts_msg = SetJointEffortsRequest()
    set_joint_efforts_msg.wait.data = True
    set_joint_efforts_msg.joint_names = [
        "panda_finger_joint1",
        "panda_finger_joint2",
        "panda_joint1",
        "panda_joint2",
    ]
    set_joint_efforts_msg.joint_efforts.data = [0, 0, 0, 0]
    # set_joint_efforts_msg.joint_efforts.data = [50, 50, 50, 30]
    retval = set_joint_effort_srv.call(set_joint_efforts_msg)

    # ####### - TEST SET ARM JOINT EFFORTS - #########
    # %% /panda_control_server/panda_arm/set_joint_efforts test
    # Connect to /panda_control_server/set_joint_efforts
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_arm/set_joint_efforts' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_arm/set_joint_efforts", timeout=10
    )
    set_arm_joint_effort_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_arm/set_joint_efforts", SetJointEfforts
    )
    rospy.logdebug(
        "Connected to 'panda_control_server/panda_arm/set_joint_efforts' service!"
    )

    # Generate joint_efforts msg
    set_arm_joint_efforts_msg = SetJointEffortsRequest()
    set_arm_joint_efforts_msg.joint_names = ["panda_joint2", "panda_joint3"]
    set_arm_joint_efforts_msg.joint_efforts.data = [0, 0]
    # set_arm_joint_efforts_msg.joint_efforts.data = [0, 0, 0]
    retval = set_arm_joint_effort_srv.call(set_arm_joint_efforts_msg)
    print(retval.message)

    # ######## - TEST SET HAND EFFORTS - #########
    # %% /panda_control_server/panda_hand/set_joint_positions test
    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_hand/set_joint_efforts' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_hand/set_joint_efforts", timeout=10
    )
    set_hand_joint_effort_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_joint_efforts", SetJointEfforts
    )
    rospy.logdebug(
        "Connected to 'panda_control_server/panda_hand/set_joint_efforts' service!"
    )

    # Generate joint_efforts msg
    set_hand_joint_efforts_msg = SetJointEffortsRequest()
    set_hand_joint_efforts_msg.joint_names = [
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]
    set_hand_joint_efforts_msg.joint_efforts.data = [-0.08, 0.05]
    # set_hand_joint_efforts_msg.joint_efforts.data = [-0.08, -0.08]
    set_hand_joint_efforts_msg.wait.data = True
    retval = set_hand_joint_effort_srv.call(set_hand_joint_efforts_msg)
    print(retval.message)

    # ######## - TEST SET JOINT POSITIONS - #########
    # #%% /panda_control_server/set_joint_positions test

    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug("Connecting to '/panda_control_server/set_joint_positions' service.")
    rospy.wait_for_service("/panda_control_server/set_joint_positions", timeout=10)
    set_joint_positions_srv = rospy.ServiceProxy(
        "/panda_control_server/set_joint_positions", SetJointPositions
    )
    rospy.logdebug("Connected to 'panda_control_server/set_joint_positions' service!")

    # Generate joint_efforts msg
    set_joint_positions_msg = SetJointPositionsRequest()
    set_joint_positions_msg.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
    set_joint_positions_msg.joint_positions.data = [1.5, 2]
    # set_joint_positions_msg.joint_positions.data = [0.0, 0.0, 0.0, 1.5, 1.5, 0.0, 0.0]
    # set_joint_positions_msg.joint_positions.data = [
    #     1.5,
    #     0.0,
    #     1.0,
    #     1.5,
    #     1.5,
    #     1.0,
    #     1.0,
    #     0.02,
    #     0.02,
    # ]
    # set_joint_positions_msg.joint_positions.data = [
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.02,
    #     0.02,
    # ]
    # set_arm_joint_positions_msg.joint_positions.data = [0.0, 1.5]
    set_joint_positions_msg.wait.data = True
    # set_joint_positions_msg.joint_names = ["panda_finger_joint1", "panda_joint2"]
    retval = set_joint_positions_srv.call(set_joint_positions_msg)
    print(retval.message)

    # ######## - TEST SET ARM JOINT POSITIONS - #########
    # #%% /panda_control_server/panda_arm/set_joint_positions test

    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_arm/set_joint_positions' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_arm/set_joint_positions", timeout=10
    )
    set_arm_joint_positions_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_arm/set_joint_positions", SetJointPositions
    )
    rospy.logdebug(
        "Connected to 'panda_control_server/panda_arm/set_joint_positions' service!"
    )

    # Generate set_arm_joint_positions_msg
    set_arm_joint_positions_msg = SetJointPositionsRequest()
    set_arm_joint_positions_msg.joint_names = ["panda_joint5", "panda_joint6"]
    set_arm_joint_positions_msg.joint_positions.data = [1.5, 2]
    # set_arm_joint_positions_msg.joint_positions.data = [
    # 0.0,
    # 0.0,
    # 0.0,
    # 1.5,
    # 1.5,
    # 0.0,
    # 0.0,
    # ]
    # set_arm_joint_positions_msg.joint_positions.data = [
    #     1.5,
    #     1.0,
    #     1.0,
    #     1.5,
    #     1.5,
    #     1.0,
    #     1.0,
    # ]
    # set_arm_joint_positions_msg.joint_positions.data = [0.0, 1.5]
    set_arm_joint_positions_msg.wait.data = True
    retval = set_arm_joint_positions_srv.call(set_arm_joint_positions_msg)
    print(retval.message)

    # ######## - TEST SET HAND JOINT POSITIONS - #########
    # %% /panda_control_server/panda_hand/set_joint_positions test
    # Connect to /panda_control_server/set_joint_positions
    rospy.logdebug(
        "Connecting to '/panda_control_server/panda_hand/set_joint_positions' service."
    )
    rospy.wait_for_service(
        "/panda_control_server/panda_hand/set_joint_positions", timeout=10
    )
    set_hand_joint_positions_srv = rospy.ServiceProxy(
        "/panda_control_server/panda_hand/set_joint_positions", SetJointPositions
    )
    rospy.logdebug(
        "Connected to 'panda_control_server/panda_hand/set_joint_positions' service!"
    )

    # Generate joint_efforts msg
    set_hand_joint_positions_msg = SetJointPositionsRequest()
    set_hand_joint_positions_msg.joint_names = [
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]
    set_hand_joint_positions_msg.joint_positions.data = [0.04, 0.04]
    set_hand_joint_positions_msg.wait.data = True
    retval = set_hand_joint_positions_srv.call(set_hand_joint_positions_msg)
    print(retval.message)
