import roslaunch
import rospy

# Starting script
# package = "turtlebot3_openai_example"
# executable = "start_simulation.launch"
# node = roslaunch.core.Node(package, executable)

# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()

# process = launch.launch(node)
# print(process.is_alive())
# process.stop()

## Starting launchfile

rospy.init_node("test", anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/haier/catkin_ws/src/testapi/launch/test_node.launch"]
)
launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()
