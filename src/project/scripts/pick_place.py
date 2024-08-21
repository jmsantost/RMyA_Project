#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def move_to_joint_positions(group, joint_positions):
    group.set_start_state_to_current_state()  # Actualiza el estado inicial al estado actual
    group.go(joint_positions, wait=True)
    group.stop()

def close_gripper():
    gripper_group.set_joint_value_target([0.158, -0.158, 0.158, 0.0, 0.158, -0.158, 0.158, 0.0])
    gripper_group.go(wait=True)
    gripper_group.stop()

def open_gripper():
    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)
    gripper_group.stop()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_waypoints', anonymous=True)
    
    global move_group, gripper_group
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    
    waypoints = [
        [-0.722, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, -0.757, -1, -1.5, 0.0],
        [-1.320, -0.577, 0.153, -1.121, -1.5, 0.0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0.229], 
    ]
    
    for position in waypoints:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    close_gripper()
    rospy.sleep(2)

    new_waypoints = [
       [-1.317, -0.8, 0.413, -1.323, -1.5, 0.229],  
       [0.757, -0.8, 0.413, -1.323, -1.5, 0.229],
       [0.793, -0.531, 1, -2.162, -1.504, 0.267],
      
    ]
    
    for position in new_waypoints:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Abrir la pinza
    open_gripper()
    rospy.sleep(2)

    nuevo_waypoint_1 = [
                        [0.793, -0.898, 1, -2.162, -1.504, 0.267],
                        [0, -0.898, 1, -2.162, -1.504, 0.267],
                        [0, 0,0,0,0,0] ,
                        
    ]
    for position in nuevo_waypoint_1:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)



    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
