import rospy
from geometry_msgs.msg import Pose, Vector3
from openai_ros.common.markers import SampleRegionMarker, TargetMarker
from openai_ros.task_envs.panda.markers import PuckMarker, CubeMarker
from visualization_msgs.msg import Marker
from rospy import ROSInitException
from std_msgs.msg import ColorRGBA, Header

if __name__ == "__main__":
    rospy.init_node("test_rviz_markers")

    # Create current target publisher
    rospy.logdebug("Creating target pose publisher.")
    target_pose_pub = rospy.Publisher("openai_ros/object", Marker, queue_size=10)
    rospy.logdebug("Goal target publisher created.")

    # Publish marker
    # Overwrite attributes with defaults if not supplied in the constructor
    # Pre-initialize header
    # header = Header()
    # try:  # Check if rostime was initialized
    #     header.stamp = rospy.Time.now()
    # except ROSInitException:
    #     raise Exception(
    #         "Goal marker could not be created as the ROS time is not "
    #         "initialized. Have you called init_node()?"
    #     )
    # header.frame_id = "world"
    # color = ColorRGBA()
    # color.a = 1.0
    # color.r = 1.0
    # color.g = 0.0
    # color.b = 0.0
    # scale = Vector3()
    # scale.x = 0.025
    # scale.y = 0.025
    # scale.z = 0.025
    # id = 0
    # type = Marker.CUBE
    # action = Marker.ADD
    # lifetime = rospy.Duration(-1)

    # # Make sure the position quaternion is normalized
    goal_maker_pose = Pose()
    goal_maker_pose.position.x = 0.562621
    goal_maker_pose.position.y = 0.318274
    goal_maker_pose.position.z = 0.479510

    target_marker = CubeMarker(pose=goal_maker_pose, frame_id="cube")
    while rospy.signal_shutdown:
        target_pose_pub.publish(target_marker)

    # Stop
    print("stop")
