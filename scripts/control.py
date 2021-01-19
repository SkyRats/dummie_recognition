#!/usr/bin/env python

import rospy
from mavbase.MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
import math

from dummie_recognition.msg import Running

height = 2

def run_callback(state):
    global run_state
    run_state = state

def pose_callback(pose):
    global drone_pose
    drone_pose = pose

def dist(dummie, dummie_x, dummie_y, never_found):
    if never_found:
        rospy.loginfo("NERVER FOUND")
        return 5

    else:
        dist_x = (dummie.pose_x - dummie_x)
        dist_y = (dummie.pose_y - dummie_y)

        dist_total = math.sqrt((dist_x**2 + dist_y**2))
        rospy.loginfo("calculating dist")
        return dist_total

def run():

    rospy.init_node("head")
    mav = MAV("jorge")

    run_pub = rospy.Publisher("dummie_recognition/set_running_state", Running, queue_size = 10)
    run_sub = rospy.Subscriber("dummie_recognition/set_running_state", Running, run_callback, queue_size = 1)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

    rospy.logwarn("CONTROL.PY RUNNING")
    mav.takeoff(height)
    mav.set_position(0, 0, height)

    #NEW MSG
    dummie = Running()
    dummie.search = False
    dummie.detected = False
    dummie.id = 0
    dummie.pose_x = 100
    dummie.pose_y = 100
    
    for i in range(10):
        run_pub.publish(dummie)
        mav.rate.sleep()
    
    # INICIALIZE DumDetect()
    rospy.logwarn("INICIALIZING SEARCH")
    for i in range(10):
        dummie.search = True
        run_pub.publish(dummie)
        mav.rate.sleep()
    
    goal_pose_x = 3
    goal_pose_y = 3
    dummie_x = 0
    dummie_y = 0

    never_found = True
    
    while (abs(goal_pose_x - drone_pose.pose.position.x) > 0.01) and (abs(goal_pose_y - drone_pose.pose.position.y) > 0.01):
        
        mav.set_position(goal_pose_x, goal_pose_y, height)
        
        if run_state.detected and dist(run_state, dummie_x, dummie_y, never_found) >= 1.5:
            mav.hold(3)
            rospy.logwarn('DUMMIE FOUND - ID = ' + str(run_state.id) + ' - py')
            never_found = False
            dummie_x = run_state.pose_x
            dummie_y = run_state.pose_y

        elif run_state.detected and dist(run_state, dummie_x, dummie_y, never_found) < 1.5:
            rospy.loginfo("Detected, but too close")

        else:
            rospy.loginfo("Nothing detected")

        dummie.search = True
        run_pub.publish(dummie)
        mav.rate.sleep()

    # END DumDetect()
    rospy.logwarn("ENDING SEARCH")
    dummie.search = False
    for i in range(10):
        run_pub.publish(dummie)
        mav.rate.sleep()

    mav.land()

if __name__ == "__main__":
    run()