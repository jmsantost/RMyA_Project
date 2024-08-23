#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def move_to_joint_positions(group, joint_positions):
    # Actualiza el estado inicial al estado actual del grupo de articulaciones
    group.set_start_state_to_current_state()
    # Mueve el grupo de articulaciones a las posiciones dadas
    group.go(joint_positions, wait=True)
    # Detiene cualquier movimiento residual
    group.stop()

def close_gripper():
    # Establece el objetivo de las articulaciones de la pinza para cerrarla
    gripper_group.set_joint_value_target([0.158, -0.158, 0.158, 0.0, 0.158, -0.158, 0.158, 0.0])
    # Ejecuta el movimiento para cerrar la pinza
    gripper_group.go(wait=True)
    # Detiene cualquier movimiento residual
    gripper_group.stop()

def open_gripper():
    # Establece el objetivo de la pinza en la posición "abierta"
    gripper_group.set_named_target("open")
    # Ejecuta el movimiento para abrir la pinza
    gripper_group.go(wait=True)
    # Detiene cualquier movimiento residual
    gripper_group.stop()

def main():
    # Inicializa la interfaz de MoveIt! para ROS
    moveit_commander.roscpp_initialize(sys.argv)
    # Inicializa el nodo de ROS con el nombre 'move_robot_waypoints'
    rospy.init_node('move_robot_waypoints', anonymous=True)
    
    global move_group, gripper_group
    # Define el grupo de articulaciones del brazo manipulador
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    # Define el grupo de articulaciones de la pinza
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
    # Establece los factores de escala de velocidad y aceleración máxima
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    
    # Puntos desde HOME (dónde todos los joints son 0) hasta la botella.
    hometobottle = [
        [-0.722, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, 0.0, 0.0, 0.0, 0.0],
        [-1.320, -0.616, -0.757, -1, -1.5, 0.0],
        [-1.320, -0.577, 0.153, -1.121, -1.5, 0.0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0], 
        [-1.317, -0.654, 0.413, -1.323, -1.5, 0.229], 
    ]
    
    # Mueve el brazo siguiedno la trayectoria definida de hometobottle
    for position in hometobottle:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Cierra la pinza
    close_gripper()
    rospy.sleep(2)

    # Puntos desde la botetlla hasta la caja.
    bottletobox = [
       [-1.317, -0.8, 0.413, -1.323, -1.5, 0.229],  
       [0.757, -0.8, 0.413, -1.323, -1.5, 0.229],
       [0.793, -0.531, 1, -2.162, -1.504, 0.267],
    ]
    
    # Mueve el brazo siguiedno la trayectoria definida de bottletobox
    for position in bottletobox:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Abre la pinza
    open_gripper()
    rospy.sleep(2)

    # Puntos desde Box hacia Home.
    boxtohome= [
        [0.793, -0.898, 1, -2.162, -1.504, 0.267],
        [0, -0.898, 1, -2.162, -1.504, 0.267],
        [0, 0, 0, 0, 0, 0],
    ]
    
    #Mueve el brazo siguiedno la trayectoria definida de boxtohome
    for position in boxtohome:
        move_to_joint_positions(move_group, position)
        rospy.sleep(1)

    # Cierra la interfaz de MoveIt! para ROS
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
