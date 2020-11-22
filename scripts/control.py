#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped

from cv_detection.msg import H_info

height = 2

def run():
    
    rospy.init_node("head")
    mav = MAV("jorge")
    running_state = False
    running_state_pub = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
    
    mav.takeoff(height)
    running_state = True
    running_state_pub.publish(running_state)
    mav.set_position(3, 3, height)


if __name__ == "__main__":
    run()