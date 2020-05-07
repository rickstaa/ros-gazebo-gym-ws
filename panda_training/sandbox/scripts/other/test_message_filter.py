import message_filters
import rospy
from std_msgs.msg import Float64


def callback():
    print("jan")


panda_joint1_pub = rospy.Publisher(
    "/panda_arm_joint1_position_controller/command", Float64, queue_size=10
)
panda_joint2_pub = rospy.Publisher(
    "/panda_arm_joint1_position_controller/command", Float64, queue_size=10
)

ts = message_filters.ApproximateTimeSynchronizer(
    [panda_joint1_pub, panda_joint2_pub], 10, 0.3
)
ts.registerCallback(callback)
print("ka")
