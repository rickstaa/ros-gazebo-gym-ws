import rospy

# Initialize ros node
rospy.init_node("ros_debug_test", log_level=rospy.DEBUG)

rospy.logdebug("DEBUG")
rospy.loginfo("INFO")
rospy.logwarn("WARN")
rospy.logerr("ERROR")
rospy.logfatal("FATAL")
