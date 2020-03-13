#! /usr/bin/env python
"""A simple server for sending control commands to the controllers
of the panda robot."""

# Main python imports
import sys

# ROS python imports
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

# ROS msgs and srvs
from std_msgs.msg import Float64MultiArray
from panda_training.srv import (
    setJointPositions,
    setJointPositionsResponse,
    setJointEfforts,
    setJointEffortsResponse,
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

        # Create joint group position publisher
        self._panda_arm_joint_positions_publisher = rospy.Publisher(
            "panda_arm_joint_group_position_controller/command",
            Float64MultiArray,
            queue_size=10,
        )
        self._panda_arm_joint_efforts_publisher = rospy.Publisher(
            "panda_arm_joint_group_effort_controller/command",
            Float64MultiArray,
            queue_size=10,
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
                self._panda_arm_joint_group_position_input_size = len(
                    controller.claimed_resources[0].resources
                )
            elif controller.name == "panda_arm_joint_group_effort_controller":
                self._panda_arm_joint_effort_input_size = len(
                    controller.claimed_resources[0].resources
                )

        # Create PandaControl services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_positions' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_positions" % rospy.get_name()[1:],
            setJointPositions,
            self.panda_arm_set_joint_positions_callback,
        )
        rospy.logdebug(
            "Creating '%s/panda_arm/set_joint_efforts' service." % rospy.get_name()
        )
        self.set_ee_pose_srv = rospy.Service(
            "%s/panda_arm/set_joint_efforts" % rospy.get_name()[1:],
            setJointEfforts,
            self.panda_arm_set_joint_efforts_callback,
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

    ###############################################
    # Service callback functions ##################
    ###############################################
    def panda_arm_set_joint_positions_callback(self, joint_positions):
        """Request Joint position control

        Parameters
        ----------
        joint_positions : panda_training.srv.setJointPositionsRequest
            Service request message specifying the positions for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # create service response message
        resp = setJointPositionsResponse()

        # Check input size
        if (
            len(joint_positions.joint_positions.data)
            != self._panda_arm_joint_group_position_input_size
        ):
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (
                    len(joint_positions.joint_positions.data),
                    self._panda_arm_joint_group_position_input_size,
                )
            )
            resp.success = False
            return resp

        # Fill joint position_callback message
        req = Float64MultiArray()
        req.data = joint_positions.joint_positions.data

        # Publish request
        self._panda_arm_joint_positions_publisher.publish(req)
        resp.success = True
        return resp.success

    def panda_arm_set_joint_efforts_callback(self, joint_efforts):
        """Request Joint effort control

        Parameters
        ----------
        joint_effort : panda_training.srv.setJointEffortsRequest
            Service request message specifying the efforts for the robot arm joints.

        Returns
        -------
        panda_training.srv.setJointPositionsResponse
            Service response.
        """

        # create service response message
        resp = setJointEffortsResponse()

        # Check input size
        if (
            len(joint_efforts.joint_efforts.data)
            != self._panda_arm_joint_effort_input_size
        ):
            rospy.logwarn(
                "You specified %s joint positions while the panda_arm joint "
                "only takes %s joint positions."
                % (
                    len(joint_efforts.joint_efforts.data),
                    self._panda_arm_joint_effort_input_size,
                )
            )
            resp.success = False
            return resp

        # Fill joint position_callback message
        req = Float64MultiArray()
        req.data = joint_efforts.joint_efforts.data

        # Publish request
        self._panda_arm_joint_efforts_publisher.publish(req)

        # Return service response
        resp.success = True
        return resp.success


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate ROS nodetypes
    rospy.init_node("panda_control_server")

    # Start action sever
    server = PandaControlServer()
    rospy.spin()  # Maintain the service open.
