#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the panda robot."""

# Main python imports
import sys
import numpy as np
from functions import controller_list_array_2_dict
from group_publisher import GroupPublisher

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from panda_training.srv import (
    setJointPositions,
    setJointPositionsResponse,
    setJointEfforts,
    setJointEffortsResponse,
)
from controller_manager_msgs.srv import ListControllers, ListControllersRequest

# Script variables
POSITION_CONTROLLERS = [
    "panda_arm_joint1_position_controller",
    "panda_arm_joint2_position_controller",
    "panda_arm_joint3_position_controller",
    "panda_arm_joint4_position_controller",
    "panda_arm_joint5_position_controller",
    "panda_arm_joint6_position_controller",
    "panda_arm_joint7_position_controller",
]
EFFORT_CONTROLLERS = [
    "panda_arm_joint1_effort_controller",
    "panda_arm_joint2_effort_controller",
    "panda_arm_joint3_effort_controller",
    "panda_arm_joint4_effort_controller",
    "panda_arm_joint5_effort_controller",
    "panda_arm_joint6_effort_controller",
    "panda_arm_joint7_effort_controller",
]


#################################################
# Joint Group Position Controller class #########
#################################################
class PandaControlServer(object):
    def __init__(self, connection_timeout=10):
        """Initializes the PandaControlServer object

        Parameters
        ----------
        connection_timeout : int, optional
            The timeout for connecting to the controller_manager services,
            by default 3 sec.
        """

        # Create class attributes
        self.joint_positions_setpoint = []
        self.joint_efforts_setpoint = []
        self.joint_state_topic = "/joint_states"
        self.joint_positions_threshold = 0.01
        self.joint_positions_grad_threshold = 0.01
        self.joint_efforts_threshold = 0.01
        self.joint_efforts_grad_threshold = 0.01
        self.wait_till_done_timeout = rospy.Duration(3)

        # Create joint position publishers
        self._arm_position_pub = GroupPublisher()
        for position_controller in POSITION_CONTROLLERS:
            self._arm_position_pub.append(
                rospy.Publisher(
                    "%s/command" % position_controller, Float64, queue_size=10
                )
            )

        # Create joint effort publishers
        self._arm_effort_pub = GroupPublisher()
        for effort_controller in EFFORT_CONTROLLERS:
            self._arm_effort_pub.append(
                rospy.Publisher(
                    "%s/command" % effort_controller, Float64, queue_size=10
                )
            )

        # Retrieve current robot joint state and effort information
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self.joint_state_topic, JointState, timeout=1.0
                )

                # Set joint setpoint to current position
                self.joint_positions_setpoint = list(self.joints.position)
                self.joint_efforts_setpoint = list(self.joints.effort)
            except ROSException:
                rospy.logwarn(
                    "Current /joint_states not ready yet, retrying for getting %s"
                    % self.joint_state_topic
                )

        # Create joint_state subscriber
        self.joint_states_sub = rospy.Subscriber(
            self.joint_state_topic, JointState, self._joints_callback
        )

        # Connect to controller_manager services
        try:

            # Connect to list service
            rospy.logdebug(
                "Connecting to '/controller_manager/list_controllers' service."
            )
            rospy.wait_for_service(
                "/controller_manager/list_controllers", timeout=connection_timeout
            )
            self.list_controllers_client = rospy.ServiceProxy(
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

        # Retrieve informations about the controllers
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )

        # Set control input spaces
        self._arm_joint_position_input_size = len(
            self._controllers["panda_arm_joint_group_position_controller"]
            .claimed_resources[0]
            .resources
        )
        self._arm_joint_effort_input_size = len(
            self._controllers["panda_arm_joint_group_effort_controller"]
            .claimed_resources[0]
            .resources
        )

        # Create PandaControl services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            setJointPositions,
            self.arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            setJointEfforts,
            self.arm_set_joint_efforts_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

    ###############################################
    # panda control member functions ##############
    ###############################################
    def _wait_till_done(self, control_type, timeout=None):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Parameters
        ----------
        control_type : str
            The type of control that is being executed and on which we should wait.
            Options are 'effort_control' and 'position_control'.
        connection_timeout : int, optional
            The timeout when waiting for the control to be done.
        """

        # Get set input arguments
        if not None:  # If not supplied
            timeout = self.wait_till_done_timeout

        # Wait till robot positions/efforts are not changing anymore
        # NOTE: We have to use the std to determine wheter the control was finished
        # as the velocity in the joint_states topic is wrong (see issue 14)
        # TODO: Can be changed to /joint_states.velocity if #14 is fixed.
        timeout_time = rospy.get_rostime() + timeout
        positions_buffer = np.full((2, len(self.joints.position)), np.nan)
        positions_grad = np.full((2, len(self.joints.position)), np.nan)
        efforts_buffer = np.full((2, len(self.joints.effort)), np.nan)
        efforts_grad = np.full((2, len(self.joints.effort)), np.nan)
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:

            # Wait till joint positions are within range or arm not changing anymore
            if control_type.lower() == "position_control":

                # Add state to position to buffer
                positions_buffer = np.append(
                    positions_buffer, [self.joints.position], axis=0
                )
                np.delete(positions_buffer, 0, axis=0)  # Delete oldest entry
                positions_grad = np.gradient(positions_buffer, axis=0)

                # Check if joint states are within setpoints
                if (
                    np.linalg.norm(
                        np.array(self.joints.position)
                        - np.array(self.joint_positions_setpoint)
                    )
                    <= self.joint_positions_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self.joint_positions_grad_threshold
                            and val != 0.0
                        )
                        for val in positions_grad[-1]
                    ]
                ):  # Check if difference norm is within threshold
                    break

            # Check if joint effort states are within setpoint
            elif control_type.lower == "effort_control":

                # Add state to position to buffer
                efforts_buffer = np.append(
                    efforts_buffer, [self.joints.position], axis=0
                )
                np.delete(efforts_buffer, 0, axis=0)  # Delete oldest entry
                efforts_grad = np.gradient(efforts_buffer, axis=0)

                if (
                    np.linalg.norm(
                        np.array(self.joints.effort)
                        - np.array(self.joint_efforts_setpoint)
                    )
                    <= self.joint_efforts_threshold
                ) or all(
                    [
                        (
                            np.abs(val) <= self.joint_efforts_grad_threshold
                            and val != 0.0
                        )
                        for val in efforts_grad[-1]
                    ]
                ):  # Check if difference norm is within threshold
                    break
            else:
                rospy.loginfo(
                    "Not waiting for control to be completed as '%s' is not "
                    "a vallid control type"
                )

    ###############################################
    # Service callback functions ##################
    ###############################################
    def arm_set_joint_positions_callback(self, joint_positions_req):
        """Request Joint position control

        Parameters
        ----------
        joint_positions_req : panda_training.srv.setJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Check if required controllers are running
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            position_controller
            for position_controller in POSITION_CONTROLLERS
            if self._controllers[position_controller].state != "running"
        ]
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Joint positions command send but probably not executed as the %s"
                "position controllers are not running." % missing_controllers
            )

        # Save position control setpoint
        self.joint_positions_setpoint[0:2] = list(
            self.joints.position[0:2]
        )  # Set gripper positions
        self.joint_positions_setpoint[2:] = list(
            joint_positions_req.joint_positions.data
        )  # Arm positions

        # create service response message
        resp = setJointPositionsResponse()

        # Check input size
        if (
            len(joint_positions_req.joint_positions.data)
            != self._arm_joint_position_input_size
        ):
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (
                    len(joint_positions_req.joint_positions.data),
                    self._arm_joint_position_input_size,
                )
            )
            resp.success = False
            return resp

        # Publish request
        pub_msg = [Float64(item) for item in joint_positions_req.joint_positions.data]
        self._arm_position_pub.publish(pub_msg)

        # Wait till control is finished or timeout has been reached
        if joint_positions_req.wait.data:
            self._wait_till_done("position_control")

        # Return success message
        resp.success = True
        return resp.success

    def arm_set_joint_efforts_callback(self, joint_efforts_req):
        """Request Joint effort control

        Parameters
        ----------
        joint_efforts_req : panda_training.srv.setJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # Check if required controllers are running
        self._controllers = controller_list_array_2_dict(
            self.list_controllers_client.call(ListControllersRequest())
        )
        missing_controllers = [
            effort_controller
            for effort_controller in EFFORT_CONTROLLERS
            if self._controllers[effort_controller].state != "running"
        ]
        if len(missing_controllers) >= 1:
            rospy.logwarn(
                "Joint effort command send but probably not executed as the %s"
                "effort controllers are not running." % missing_controllers
            )

        # Save effort control setpoint
        self.joint_efforts_setpoint[0:2] = list(
            self.joints.effort[0:2]
        )  # Set gripper efforts
        self.joint_efforts_setpoint[2:] = list(
            joint_efforts_req.joint_efforts.data
        )  # Arm efforts

        # create service response message
        resp = setJointEffortsResponse()

        # Check input size
        if (
            len(joint_efforts_req.joint_efforts.data)
            != self._arm_joint_effort_input_size
        ):
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (
                    len(joint_efforts_req.joint_efforts.data),
                    self._arm_joint_effort_input_size,
                )
            )
            resp.success = False
            return resp

        # Publish request
        pub_msg = [Float64(item) for item in joint_efforts_req.joint_efforts.data]
        self._arm_effort_pub.publish(pub_msg)

        # Wait till control is finished or timeout has been reached
        if joint_efforts_req.wait.data:
            self._wait_till_done(control_type="effort_control")

        # Return service response
        resp.success = True
        return resp.success

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """

        # Update joint_states
        self.joints = data


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate ROS nodetypes
    rospy.init_node("panda_control_server")

    # Start control server
    server = PandaControlServer()
    rospy.spin()  # Maintain the service open.
