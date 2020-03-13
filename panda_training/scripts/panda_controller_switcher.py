"""Class used to switch between the ROS controllers which are available
in the panda_training package. The currently available controllers are:
 - panda_arm_controller: Controls panda arm joints based on joint
   trajectory.
 - panda_arm_joint_group_effort_controller: Controls panda arm joints based on joint
   effort array.
 - panda_arm_joint_group_position_controller: Controls panda_arm joints based on joint
   position array.
"""

# Main python imports
import sys

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from controller_manager_msgs.srv import (
    SwitchController,
    SwitchControllerRequest,
    ListControllers,
    ListControllersRequest,
)

# Global variables
ARM_CONTROLLERS = [
    "panda_arm_controller",
    "panda_arm_joint_group_effort_controller",
    "panda_arm_joint_group_position_controller",
]
HAND_CONTROLLERS = ["panda_hand_controller"]


#################################################
# Panda Controller Switcher #####################
#################################################
class PandaControllerSwitcher(object):
    def __init__(
        self,
        arm_controllers=ARM_CONTROLLERS,
        hand_controllers=HAND_CONTROLLERS,
        connection_timeout=10,
    ):
        """Initializes the MoveitPlannerServer object.

        Parameters
        ----------
        arm_controllers : list, optional
            A list containing the currently used panda arm controllers,
            by default ARM_CONTROLLERS.
        hand_controllers : list, optional
            A list containing the currently used panda hand controllers,
            by default HAND_CONTROLLERS.
        connection_timeout : str, optional
            The timeout for connecting to the controller_manager services,
            by default 3 sec.
        """

        # Retrieve constructor variables
        self.arm_controllers = arm_controllers
        self.hand_controllers = hand_controllers

        # Connect to controller_manager services
        try:

            # Connect to switch service
            rospy.logdebug(
                "Connecting to '/controller_manager/switch_controller' service."
            )
            rospy.wait_for_service(
                "/controller_manager/switch_controller", timeout=connection_timeout
            )
            self.switch_controller_client = rospy.ServiceProxy(
                "/controller_manager/switch_controller", SwitchController
            )
            rospy.logdebug(
                "Connected to '/controller_manager/switch_controller' service!"
            )

            # Connect to list service
            rospy.logdebug(
                "Connecting to '/controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "/controller_manager/list_controllers", timeout=connection_timeout
            )
            self.list_controller_client = rospy.ServiceProxy(
                "/controller_manager/list_controllers", ListControllers
            )
            rospy.logdebug(
                "Connected to '/controller_manager/list_controllers' service!"
            )

        except (rospy.ServiceException, ROSException, ROSInterruptException) as e:
            rospy.logerr(
                "Shutting down '%s' because no connection could be established "
                "with the '%s' service and this service is needed "
                "when using 'joint_position_control'."
                % (
                    rospy.get_name(),
                    e.args[0].strip("timeout exceeded while waiting for service"),
                )
            )
            sys.exit(0)

    def list_running(self):
        """List running controllers panda_arm controllers.

        Returns
        -------
        dict
            Dictionary containing controllers that are currently running.
        """

        # Get the controller that is currently running
        controllers = self.list_controller_client.call(ListControllersRequest())

        # Check which controllers are running
        active_controllers = {"arm": [], "hand": [], "other": []}
        for controller in controllers.controller:
            if controller.name in self.arm_controllers:
                if controller.state == "running":
                    active_controllers["arm"].append(controller.name)
            elif controller.name in self.hand_controllers:
                if controller.state == "running":
                    active_controllers["hand"].append(controller.name)
            else:
                if controller.state == "running":
                    active_controllers["other"].append(controller.name)

        # Throw warning if conflicting controllers are running
        if len(active_controllers["arm"]) > 1:
            rospy.logwarn(
                "It appears the %s panda arm controllers are running at the "
                "same time. As these controllers conflict each other please "
                "check the arm_controllers that is supplied to the %s node."
                % (active_controllers["arm"], rospy.get_name())
            )

        # Return list with running controllers
        return active_controllers

    def switch(self, control_group, controller):
        """Starts a ROS controller while stopping the
        controller that is currently running.

        Parameters
        ----------
        control_group : str
            The control group of which you want the switch the running controller.
        control_group : str
            The controller you want to run.

        Returns
        -------
        bool
            Success boolean
        """

        # Validate input arguments
        if type(controller) == list:
            if len(controller) > 1:
                rospy.loginfo("Please specify a single controller you want to run.")
            else:
                controller = controller[0]
        if type(control_group) == list:
            if len(control_group) > 1:
                rospy.loginfo(
                    "Please specify a single control group in which you want "
                    "to switch the controller."
                )
            else:
                control_group = control_group[0]

        # Get active controllers
        controllers_running = self.list_running()

        # Generate switch controller msg
        rospy.loginfo("Switching to %s controller." % controller)
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers.append(controller)
        try:
            switch_controller_msg.stop_controllers.extend(
                controllers_running[control_group]
            )
        except KeyError:
            rospy.logwarn(
                "The controller was not switched to %s as control group "
                " %s does not exists." % (controller, control_group)
            )
        switch_controller_msg.strictness = SwitchControllerRequest.STRICT
        switch_controller_msg.timeout = 3

        # Send switch_controller msgs
        retval = self.switch_controller_client(switch_controller_msg)
        return retval.ok


if __name__ == "__main__":

    rospy.init_node("test")
    # Create panda_control switcher
    panda_control_switcher = PandaControllerSwitcher()
    retval = panda_control_switcher.switch(
        control_group="arm", controller="panda_arm_joint_group_effort_controller"
    )
    print(retval)
