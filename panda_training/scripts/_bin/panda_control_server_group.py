#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the Panda robot."""

# Main python imports
import sys

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from panda_training.srv import (
    SetJointPositions,
    SetJointPositionsResponse,
    SetJointEfforts,
    SetJointEffortsResponse,
)
from controller_manager_msgs.srv import ListControllers, ListControllersRequest


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
        self.control_ready = True  # Whether the control has been finished
        self.joint_positions_setpoint = []
        self.joint_efforts_setpoint = []
        self.joint_state_topic = "/joint_states"
        self.joint_positions_threshold = 0.01
        self.wait_till_done_timeout = rospy.Duration(10)

        # Create joint group position publisher
        self._arm_joint_positions_publisher = rospy.Publisher(
            "panda_arm_joint_group_position_controller/command",
            Float64MultiArray,
            queue_size=10,
        )
        self._arm_joint_efforts_publisher = rospy.Publisher(
            "/panda_arm_joint_group_effort_controller/command",
            Float64MultiArray,
            queue_size=10,
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

            # Connect to switch service
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
        req = ListControllersRequest()
        self._controllers = self.list_controllers_client.call(req)

        # Set control input spaces
        for controller in self._controllers.controller:
            if controller.name == "panda_arm_joint_group_position_controller":
                self._arm_joint_group_position_input_size = len(
                    controller.claimed_resources[0].resources
                )
            elif controller.name == "panda_arm_joint_group_effort_controller":
                self._arm_joint_effort_input_size = len(
                    controller.claimed_resources[0].resources
                )

        # Create PandaControl services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            SetJointPositions,
            self.arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            SetJointEfforts,
            self.arm_set_joint_efforts_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

    ###############################################
    # Panda control member functions ##############
    ###############################################
    def wait_till_done(self, timeout=None):
        """Wait control is finished. Meaning the robot state is within range of the
        Joint position and joint effort setpoints.

        Parameters
        ----------
        connection_timeout : int, optional
            The timeout when waiting for the control to be done, by default
            self.wait_till_done_timeout
        """

        # Get set input arguments
        if not None:  # If not supplied
            timeout = self.wait_till_done_timeout

        # Wait till robot is within threshold
        timeout_time = rospy.get_rostime() + timeout
        while not self.control_ready and rospy.get_rostime() < timeout_time:
            print("sleep")
        else:
            return True

    ###############################################
    # Service callback functions ##################
    ###############################################
    def arm_set_joint_positions_callback(self, joint_positions_req):
        """Request Joint position control

        Parameters
        ----------
        joint_positions_req : panda_training.srv.SetJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

        # Save control setpoint
        self.joint_positions_setpoint = joint_positions_req.joint_positions

        # create service response message
        resp = SetJointPositionsResponse()

        # Check input size
        if (
            len(joint_positions_req.joint_positions)
            != self._arm_joint_group_position_input_size
        ):
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (
                    len(joint_positions_req.joint_positions),
                    self._arm_joint_group_position_input_size,
                )
            )
            resp.success = False
            return resp

        # Fill joint position_callback message
        req = Float64MultiArray()
        req.data = list(joint_positions_req.joint_positions)

        # Publish request
        self._arm_joint_positions_publisher.publish(req)

        # Wait till control is finished
        if joint_positions_req.wait:
            self.wait_till_done()

        # Return success message
        resp.success = True
        return resp.success

    def arm_set_joint_efforts_callback(self, joint_efforts):
        """Request Joint effort control

        Parameters
        ----------
        joint_effort : panda_training.srv.SetJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.SetJointPositionsResponse
            Service response.
        """

        # Save control setpoint
        self.joint_efforts_setpoint = joint_efforts

        # create service response message
        resp = SetJointEffortsResponse()

        # Check input size
        if len(joint_efforts.joint_efforts) != self._arm_joint_effort_input_size:
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (len(joint_efforts.joint_efforts), self._arm_joint_effort_input_size,)
            )
            resp.success = False
            return resp

        # Fill joint position_callback message
        req = Float64MultiArray()
        req.data = joint_efforts.joint_efforts

        # Publish request
        self._arm_joint_efforts_publisher.publish(req)

        # Return service response
        resp.success = True
        return resp.success

    def _joints_callback(self, data):
        """Callback function for the joint data subscriber.
        """

        # Update joint_states
        self.joints = data

        # Check if joint states are within setpoints
        if (
            [
                position - self.joint_positions_threshold
                for position in self.joint_positions_setpoint
            ]
            <= list(self.joints.position)
            <= [
                position + self.joint_positions_threshold
                for position in self.joint_positions_setpoint
            ]
        ):
            print("ready")
            self.control_ready = True
            rospy.sleep(1)
        else:
            print("not ready")
            self.control_ready = False
            rospy.sleep(1)


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate ROS nodetypes
    rospy.init_node("panda_control_server")

    # Start action sever
    server = PandaControlServer()
    rospy.spin()  # Maintain the service open.
