#! /usr/bin/env python
"""Panda moveit server
A ros server that creates a number of Moveit services which can be used
to control the Panda robot or retrieve sensor data for the robot.
"""

# Main python imports
import sys

# ROS python imports
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

# ROS msgs and srvs
import moveit_msgs.msg
import geometry_msgs.msg
from panda_training.srv import (
    GetEe,
    GetEeResponse,
    GetEePose,
    GetEePoseResponse,
    GetEeRpy,
    GetEeRpyResponse,
    SetEe,
    SetEeResponse,
    SetEePose,
    SetEePoseResponse,
    SetJointPose,
    SetJointPoseResponse,
)


#################################################
# Moveit Planner Server class ###################
#################################################
class MoveitPlannerServer(object):
    def __init__(self, robot_EE_link="panda_link8", robot_move_group="panda_arm"):
        """Initializes the MoveitPlannerServer object.

        Parameters
        ----------
        robot_EE_link : str, optional
            The end effector you want moveit to use when controlling
            the panda_arm group by default "panda_link8".
        """

        # Initiate member variables
        self._move_group_name = robot_move_group

        # Initialize Moveit/Robot/Scene and group commanders
        rospy.logdebug("Initialize Moveit Robot/Scene and group commanders.")
        try:

            # NOTE: Fix arguments to solve `C++ converter` error when debugging in ptvsd
            cleaned_args = [
                a for a in sys.argv if not a.endswith("my_script_name_here.py")
            ]
            moveit_commander.roscpp_initialize(cleaned_args)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group = moveit_commander.MoveGroupCommander(self._move_group_name)
        except Exception as e:
            # Shut down node on error if
            # Robot_description not found
            if "invalid robot mode" in e.args[0]:
                rospy.logerr(
                    "Shutting down '%s' because robot_description was not found."
                    % rospy.get_name()
                )
                sys.exit(0)
            # Move group not found
            elif "Group '%s' was not found." % self._move_group_name in e.args[0]:
                rospy.logerr(
                    "Shutting down '%s' because move group '%s' was not found."
                    % (rospy.get_name(), self._move_group_name)
                )
                sys.exit(0)
            else:
                rospy.logerr(
                    "Shutting down '%s' because %s" % (rospy.get_name(), e.message)
                )
                sys.exit(0)

        # Set end effector link
        self.move_group.set_end_effector_link(robot_EE_link)

        # Create rviz trajectory publisher
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

        # Create MoveitPlannerServer services
        rospy.loginfo("Creating '%s' services." % rospy.get_name())
        rospy.logdebug("Creating '%s/set_ee_pose' service." % rospy.get_name())
        self.set_ee_pose_srv = rospy.Service(
            "%s/set_ee_pose" % rospy.get_name()[1:],
            SetEePose,
            self.set_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/set_joint_pose' service." % rospy.get_name())
        self.set_joint_traj_srv = rospy.Service(
            "%s/set_joint_pose" % rospy.get_name()[1:],
            SetJointPose,
            self.set_joint_pose_callback,
        )
        rospy.logdebug("Creating '%s/get_ee_pose' service." % rospy.get_name())
        self.get_ee_pose_srv = rospy.Service(
            "%s/get_ee_pose" % rospy.get_name()[1:],
            GetEePose,
            self.get_ee_pose_callback,
        )
        rospy.logdebug("Creating '%s/get_ee_rpy' service." % rospy.get_name())
        self.get_ee_rpy_srv = rospy.Service(
            "%s/get_ee_rpy" % rospy.get_name()[1:], GetEeRpy, self.get_ee_rpy_callback
        )
        rospy.logdebug("Creating '%s/get_ee' service." % rospy.get_name())
        self.get_ee = rospy.Service(
            "%s/get_ee" % rospy.get_name()[1:], GetEe, self.get_ee_callback
        )
        rospy.logdebug("Creating '%s/set_ee' service." % rospy.get_name())
        self.set_ee = rospy.Service(
            "%s/set_ee" % rospy.get_name()[1:], SetEe, self.set_ee_callback
        )
        rospy.loginfo("'%s' services created successfully." % rospy.get_name())

        # Initiate service msgs
        self.pose_target = geometry_msgs.msg.Pose()

    ###############################################
    # Helper functions ############################
    ###############################################
    def execute(self):
        """Plan and execute a trajectory/pose or orientation setpoints"""

        # Plan and execute
        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)

    def link_exists(self, link_name):
        """Function checks whether a given link exists in the robot_description.

        Parameters
        ----------
        link_name : str
            Name of link you want to check.
        """
        return link_name in self.robot.get_link_names()

    ###############################################
    # Service callback functions ##################
    ###############################################
    def set_ee_pose_callback(self, request):
        """Request the panda arm to control to a given end effector
        (EE) pose.

        Parameters
        ----------
        request : geometry_msgs.msg.Pose
            The trajectory you want the EE to follow.

        Returns
        -------
        panda_train.srv.SetEePoseResponse
            Response message containing (success bool, message).
        """

        # Fill trajectory message
        rospy.logdebug("Setting ee pose.")
        response = SetEePoseResponse()
        self.pose_target.orientation.w = request.pose.orientation.w
        self.pose_target.position.x = request.pose.position.x
        self.pose_target.position.y = request.pose.position.y
        self.pose_target.position.z = request.pose.position.z

        # Send trajectory message and return response
        try:
            self.move_group.set_pose_target(self.pose_target)
            self.execute()
        except MoveItCommanderException as e:
            rospy.logwarn(e.message)
            response.success = True
            response.message = e.message

        # Return success message
        response.success = True
        response.message = "Everything went OK"
        return response

    def set_joint_pose_callback(self, request):
        """Request the panda arm to control to a given joint pose.

        Parameters
        ----------
        request : geometry_msgs.msg.Pose
            The joint poses you want to control the joints to.

        Returns
        -------
        panda_train.srv.SetJointPoseResponse
            Response message containing (success bool, message).
        """

        # Create response message
        rospy.logdebug("Setting joint position targets.")
        response = SetJointPoseResponse()

        # Retrieve current joint positions
        joint_positions = self.move_group.get_current_joint_values()

        # Validate request
        if len(request.point.positions) != len(joint_positions):
            rospy.logwarn(
                "Joint position not set as %s joint positions were specified "
                "while %s joint positions are required."
                % (len(request.point.positions), len(joint_positions))
            )
            response.success = False
            response.message = "Joint positions setpoint length invalid."
            return response

        # Log setpoint information
        rospy.logdebug("Current joint positions: %s" % joint_positions)
        rospy.logdebug("Joint position setpoint: %s" % list(request.point.positions))

        # Set joint positions setpoint, execute setpoint and return response
        try:
            self.move_group.set_joint_value_target(list(request.point.positions))
        except MoveItCommanderException:
            rospy.logwarn(
                "Setting joint targets failed since joint position "
                "were not within bounds."
            )
            response.success = False
            response.message = "Joint setpoint not within bounds."
            return response
        rospy.logdebug("Executing joint positions setpoint.")
        self.execute()
        response.success = True
        response.message = "Everything went OK"
        return response

    def get_ee_pose_callback(self, request):
        """Request end effector pose.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            The current end effector pose.
        """

        # Retrieve and return gripper pose
        rospy.logdebug("Retrieving ee pose.")
        gripper_pose = self.move_group.get_current_pose()
        gripper_pose_res = GetEePoseResponse()
        gripper_pose_res = gripper_pose.pose
        return gripper_pose_res

    def get_ee_rpy_callback(self, request):
        """Request current end effector (EE) orientation.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        panda_train.srv.GetEeResponse
            Response message containing containing the roll (z), yaw (y), pitch (x)
            euler angles.
        """

        # Retrieve and return gripper orientation
        rospy.logdebug("Retrieving ee orientation.")
        gripper_rpy = self.move_group.get_current_rpy()
        gripper_rpy_res = GetEeRpyResponse()
        gripper_rpy_res.r = gripper_rpy[0]
        gripper_rpy_res.y = gripper_rpy[1]
        gripper_rpy_res.p = gripper_rpy[2]
        return gripper_rpy_res

    def get_ee_callback(self, request):
        """Request end effector (EE) name.

        Parameters
        ----------
        request : std_srvs.srv.Empty
            Empty request.

        Returns
        -------
        panda_train.srv.GetEeResponse
            Response message containing the name of the current EE.
        """

        # Return EE name
        rospy.logdebug("Retrieving ee name.")
        response = GetEeResponse()
        response.ee_name = self.move_group.get_end_effector_link()
        return response

    def set_ee_callback(self, request):
        """Request end effector (EE) change.

        Parameters
        ----------
        request : panda_train.srv.SetEe
            Request message containing the name of the end effector you want to be set.

        Returns
        -------
        panda_train.srv.SetEeResponse
            Response message containing (success bool, message).
        """

        # Set end effector and return response
        rospy.logdebug("Setting ee.")
        response = SetEeResponse()
        if self.link_exists(request.ee_name):  # Check if vallid
            try:
                self.move_group.set_end_effector_link(request.ee_name)
            except MoveItCommanderException as e:
                rospy.logwarn("Ee could not be set.")
                response = False
                response.message = e.message
            response.success = True
            response.message = "Everything went OK"
        else:
            rospy.logwarn(
                "Ee could not be as %s is not a valid ee link." % request.ee_name
            )
            response.success = False
            response.message = "%s is not a valid ee link." % request.ee_name
        return response


#################################################
# Main script ###################################
#################################################
if __name__ == "__main__":

    # Initiate Moveit Planner Server
    rospy.init_node("panda_moveit_planner_server")

    # Get private parameters specified in the launch file
    try:  # Check end effector
        robot_EE_link = rospy.get_param("~end_effector")
    except KeyError:
        robot_EE_link = "panda_grip_site"

    # Start moveit planner server
    moveit_planner_server = MoveitPlannerServer(robot_EE_link=robot_EE_link)
    rospy.spin()  # Maintain the service open.
