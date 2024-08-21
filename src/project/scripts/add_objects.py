#!/usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def add_objects_to_scene():
    rospy.init_node('add_objects_to_scene', anonymous=True)
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Espera para asegurar que el nodo está completamente inicializado

    # Crear una función para aplicar la rotación
    def create_pose(x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation = Quaternion(*quat)
        return pose


    # Agregar la mesa (girada 90 grados)
    table_pose = create_pose(0.103960, -0.233287, 0.0, 0, 0, -1.57)  # Rotación de -90 grados (en radianes)
    scene.add_box("table", table_pose, size=(1.5, 0.8, 0.03))

    # Posicionar el gabinete al lado izquierdo del robot y girarlo 90 grados en Z
    cabinet_pose = create_pose(0.103960, 0.74, -0.21, 0, 1.57, 3.14)  # Ajuste en Y y rotación en Z (90 grados)
    scene.add_box("cabinet", cabinet_pose, size=(0.45, 0.45, 0.8))  # Tamaño ajustado del gabinete

    # Agregar la primera cerveza (posicionada sobre la mesa)
    beer_pose = create_pose(0.328415, -0.864340, 0.145, 0, 0, 0)
    scene.add_box("beer", beer_pose, size=(0.07, 0.07, 0.23))

    # Agregar la segunda cerveza (posicionada sobre la mesa)
    beer_0_pose = create_pose(-0.112475, -0.842375, 0.145, 0, 0, 0)
    scene.add_box("beer_0", beer_0_pose, size=(0.07, 0.07, 0.23))

    rospy.loginfo("Objects added to the scene")
    rospy.sleep(2)  # Espera para permitir que los objetos se agreguen

if __name__ == '__main__':
    try:
        add_objects_to_scene()
    except rospy.ROSInterruptException:
        pass