#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
rospy.sleep(2)
def is_gazebo_ready():
    """Verifica si Gazebo está listo verificando el estado de un modelo."""
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_model_state('ground_plane', '')
        if resp.success:
            return True
    except rospy.ServiceException:
        pass
    return False

def unpause_gazebo():
    rospy.init_node('unpause_gazebo', anonymous=True)
    
    # Esperar a que Gazebo esté completamente cargado
    while not is_gazebo_ready():
        rospy.loginfo("Waiting for Gazebo to be ready...")
        rospy.sleep(1)
    
    rospy.loginfo("Gazebo is ready. Unpausing physics...")
    
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
        rospy.loginfo("Gazebo unpaused successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to unpause Gazebo: %s" % e)

if __name__ == '__main__':
    unpause_gazebo()
