"""Class used for displaying a gasp goal marker in rviz. This class overloads the "
the visualization_msgs.msgs.Marker class in order to pre-initialize some of its "
attributes.
"""

# Main python 2/3 compatibility imports
from builtins import super

# Main python imports
import sys

# ROS python imports
import rospy
from rospy.exceptions import ROSInitException

# ROS msgs and srvs
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose


#################################################
# GoalMarker ####################################
#################################################
class GoalMarker(Marker):
    """Class used to create an rviz goal marker.
    """

    def __init__(self, *args, **kwds):
        """Initialize GoalMarker object
        """

        # Setup superclass attributes
        super().__init__()

        # Pre-initialize header
        header = Header()
        try:  # Check if rostime was initialized
            header.stamp = rospy.Time.now()
        except ROSInitException:
            raise Exception(
                "Goal markers could not be created as the ROS time is not initialized. "
                "Have you called init_node()?"
            )
            sys.exit(0)
        header.frame_id = "world"

        # Pre-initialize marker class attributes
        self.id = 0
        self.type = Marker.SPHERE
        self.action = Marker.ADD
        self.color.a = 1.0
        self.color.r = 1.0
        self.color.g = 0.0
        self.color.b = 0.0
        self.scale.x = 0.025
        self.scale.y = 0.025
        self.scale.z = 0.025
        self.lifetime = rospy.Duration(-1)
        self.header = header

        # Apply class input arguments
        super().__init__(*args, **kwds)
