#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def unpause_gazebo():
    rospy.init_node('unpause_gazebo', anonymous=True)
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
        rospy.loginfo("Gazebo unpaused successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to unpause Gazebo: %s" % e)

if __name__ == '__main__':
    unpause_gazebo()
