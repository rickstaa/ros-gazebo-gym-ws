#! /usr/bin/env python
"""Panda moveit server
A ros server that creates a number of Moveit services which can be used
to control the Panda robot or retrieve sensor data for the robot.
"""

# Main python imports
import sys
import copy

# ROS python imports
import rospy
import moveit_commander

# Ros msgs and srvs
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from panda_reach_training.srv import (
    getEePose,
    getEePoseResponse,
    getEeRpy,
    getEeRpyResponse,
    setEePose,
    setEePoseResponse,
    setJointPose,
    setJointPoseResponse,
)


#################################################
# MoveitPlannerServer class #####################
################################################
class MoveitPlannerServer(object):
    def __init__(self, end_effector="panda_link8"):
        """

        Parameters
        ----------
        end_effector : str, optional
            The end effector you want moveit to use when controlling
            the panda_arm group by default "panda_link8".
        """
        # Initialize Moveit/Robot/Scene and group commanders
        # moveit_commander.roscpp_initialize([])  # NOTE: Uncomment to debug
        moveit_commander.roscpp_initialize(sys.argv)  # NOTE: Comment to debug
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

        # Set end effector link
        self.move_group.set_end_effector_link(end_effector)

        # Create rviz trajectory publisher
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

        # Create MoveitPlannerServer services
        self.set_ee_pose_srv = rospy.Service(
            "%s/set_ee_pose" % rospy.get_name()[1:],
            setEePose,
            self.set_ee_pose_callback,
        )
        self.set_joint_traj_srv = rospy.Service(
            "%s/set_joint_pose" % rospy.get_name()[1:],
            setJointPose,
            self.set_joint_pose_callback,
        )
        self.get_ee_pose_srv = rospy.Service(
            "%s/get_ee_pose" % rospy.get_name()[1:],
            getEePose,
            self.get_ee_pose_callback,
        )
        self.get_ee_rpy_srv = rospy.Service(
            "%s/get_ee_rpy" % rospy.get_name()[1:], getEeRpy, self.get_ee_rpy_callback
        )

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
        bool
            Success bool.
        """

        # Fill trajectory message
        self.pose_target.orientation.w = request.pose.orientation.w
        self.pose_target.position.x = request.pose.position.x
        self.pose_target.position.y = request.pose.position.y
        self.pose_target.position.z = request.pose.position.z

        # Send trajectory message and return response
        self.move_group.set_pose_target(self.pose_target)
        self.execute()
        response = setEePoseResponse()
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
        bool
            Success bool.
        """

        # Fill joint trajectory message
        self.group_variable_values = self.move_group.get_current_joint_values()
        print("Group Vars:")
        print(self.group_variable_values)
        print("Point:")
        print(request.point.positions)
        self.group_variable_values[0] = request.point.positions[0]
        self.group_variable_values[1] = request.point.positions[1]
        self.group_variable_values[2] = request.point.positions[2]
        self.group_variable_values[3] = request.point.positions[3]
        self.group_variable_values[4] = request.point.positions[4]
        self.group_variable_values[5] = request.point.positions[5]
        self.group_variable_values[6] = request.point.positions[6]

        # Send trajectory message and return response
        self.move_group.set_joint_value_target(self.group_variable_values)
        self.execute()
        response = setJointPoseResponse()
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
        gripper_pose = self.move_group.get_current_pose()
        gripper_pose_res = getEePoseResponse()
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
        list
            List containing the roll (z), yaw (y), pitch (x) euler angles.
        """

        # Retrieve and return gripper orientation
        gripper_rpy = self.move_group.get_current_rpy()
        gripper_rpy_res = getEeRpyResponse()
        gripper_rpy_res.r = gripper_rpy[0]
        gripper_rpy_res.y = gripper_rpy[1]
        gripper_rpy_res.p = gripper_rpy[2]
        return gripper_rpy_res


#################################################
# Main script####################################
#################################################
if __name__ == "__main__":

    # Initiate Moveit Planner Server
    rospy.init_node("panda_moveit_planner_server")

    # Get private parameters specified in the launch file
    try:  # Check end effector
        end_effector = rospy.get_param("~end_effector")
    except KeyError:
        end_effector = "panda_grip_site"

    # Start moveit planner server
    moveit_planner_server = MoveitPlannerServer(end_effector=end_effector)
    rospy.spin()  # Maintain the service open.
