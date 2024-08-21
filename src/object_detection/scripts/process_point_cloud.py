#!/usr/bin/env python3

import rospy
import struct
import sys
import moveit_commander
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def point_cloud_callback(msg):
    points = []  # Lista para almacenar los puntos filtrados

    for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        rgb = point[3]
        s = struct.unpack('>I', struct.pack('>f', rgb))[0]
        r = (s >> 16) & 0x0000ff
        g = (s >> 8) & 0x0000ff
        b = (s) & 0x0000ff

        if 1.8 < point[2] < 2.2 and 50 <= r <= 100 and 50 <= g <= 100 and 50 <= b <= 100:
            points.append((point[0], point[1], point[2]))

    if points:
        centroid_x = sum(p[0] for p in points) / len(points)
        centroid_y = sum(p[1] for p in points) / len(points)
        centroid_z = sum(p[2] for p in points) / len(points)

        rospy.loginfo("Centroide: x={:.2f}, y={:.2f}, z={:.2f}".format(centroid_x, centroid_y, centroid_z))

        # Planificación del movimiento hacia el centroide
        move_group = moveit_commander.MoveGroupCommander("manipulator")

        # Aumenta el tiempo de planificación
        move_group.set_planning_time(10)

        # Configura la tolerancia para el objetivo (opcional)
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        # Define el objetivo
        pose_target = move_group.get_current_pose().pose
        pose_target.position.x = centroid_x
        pose_target.position.y = centroid_y
        pose_target.position.z = centroid_z

        move_group.set_pose_target(pose_target)

        # Intentar planificar y ejecutar
        plan = move_group.go(wait=True)

        if plan:
            rospy.loginfo("Movimiento completado con éxito.")
        else:
            rospy.logwarn("No se encontró una solución para el movimiento.")

        move_group.stop()
        move_group.clear_pose_targets()

def main():
    rospy.init_node('pick_place_node')
    moveit_commander.roscpp_initialize(sys.argv)

    # Esperar a que los servicios estén listos
    rospy.loginfo("Esperando a que los servicios de MoveIt! estén listos...")
    rospy.wait_for_service('compute_ik')  # O algún otro servicio relevante para tu configuración
    rospy.loginfo("Servicios de MoveIt! listos.")

    rospy.Subscriber('/rgbd_camera/depth/points', PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
