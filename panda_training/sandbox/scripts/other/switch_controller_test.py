"""Script used to test the 'controller_manager' switch controller service"""

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

if __name__ == "__main__":

    # Connect to /controller_manager/switch_controller
    rospy.logdebug("Connecting to '/controller_manager/switch_controller' service.")
    rospy.wait_for_service("/controller_manager/switch_controller", timeout=10)
    switch_control_client = rospy.ServiceProxy(
        "/controller_manager/switch_controller", SwitchController
    )
    rospy.logdebug("Connected to 'panda_moveit_planner_server/set_ee_pose' service!")

    # Generate switch controller msg start_controllers
    switch_controller_msg = SwitchControllerRequest()
    switch_controller_msg.start_controllers = ["panda_arm_controller"]
    # switch_controller_msg.stop_controllers = ["panda_arm_controller"]
    # switch_controller_msg.start_controllers = [
    #     "panda_arm_joint_group_position_controller"
    # ]
    switch_controller_msg.stop_controllers = [
        "panda_arm_joint_group_position_controller"
    ]
    switch_controller_msg.strictness = SwitchControllerRequest.STRICT
    switch_controller_msg.timeout = 10

    # Send switch control comman
    retval = switch_control_client.call(switch_controller_msg)
    print(retval)

