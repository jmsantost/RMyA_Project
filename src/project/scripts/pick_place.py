#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def move_to_joint_positions(group, joint_positions):
    # Update the initial state to the current state of the joint group
    group.set_start_state_to_current_state()
    # Move the joint group to the specified positions
    group.go(joint_positions, wait=True)
    # Stop any residual motion
    group.stop()

def close_gripper():
    # Set the target joint positions for the gripper to close it
    gripper_group.set_joint_value_target([0.158, -0.158, 0.158, 0.0, 0.158, -0.158, 0.158, 0.0])
    # Execute the motion to close the gripper
    gripper_group.go(wait=True)
    # Stop any residual motion
    gripper_group.stop()

def open_gripper():
    # Set the target of the gripper to the "open" position
    gripper_group.set_named_target("open")
    # Execute the motion to open the gripper
    gripper_group.go(wait=True)
    # Stop any residual motion
    gripper_group.stop()

def main():
    # Initialize the MoveIt! interface for ROS
    moveit_commander.roscpp_initialize(sys.argv)
    # Initialize the ROS node with the name 'move_robot_waypoints'
    rospy.init_node('move_robot_waypoints', anonymous=True)
    
    global move_group, gripper_group
    # Define the joint group for the manipulator arm
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    # Define the joint group for the gripper
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
    # Set the maximum velocity and acceleration scaling factors
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    
    # Waypoints from HOME (where all joints are 0) to BOTTLE.
    hometobottle = [
        [-0.722, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, -0.757, -1, -1.5, 0.0],
        [-1.320, -0.577, 0.153, -1.121, -1.5, 0.0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0.229], 
    ]
    
    # Move the arm following the defined trajectory from hometobottle
    for position in hometobottle:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Close the gripper
    close_gripper()
    rospy.sleep(2)

    # Waypoints from the BOTTLE to BOX.
    bottletobox = [
       [-1.317, -0.8, 0.413, -1.323, -1.5, 0.229],  
       [0.757, -0.8, 0.413, -1.323, -1.5, 0.229],
       [0.793, -0.531, 1, -2.162, -1.504, 0.267],
    ]
    
    # Move the arm following the defined trajectory from bottletobox
    for position in bottletobox:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Open the gripper
    open_gripper()
    rospy.sleep(2)

    # Waypoints from the BOX to HOME.
    boxtohome= [
        [0.793, -0.898, 1, -2.162, -1.504, 0.267],
        [0, -0.898, 1, -2.162, -1.504, 0.267],
        [0, 0, 0, 0, 0, 0],
    ]
    
    # Move the arm following the defined trajectory from boxtohome
    for position in boxtohome:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Shut down the MoveIt! interface for ROS
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
