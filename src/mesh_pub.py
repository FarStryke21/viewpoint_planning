#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys

def publish_mesh(mesh_file):
    rospy.init_node('mesh_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/mesh_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "object"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose.position.x = 1
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.mesh_resource = "package://viewpoint_planning/gazebo_models/" + mesh_file + "/mesh/" + mesh_file + ".stl"

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("No mesh file provided. Usage: rosrun viewpoint_planning publish_mesh.py <mesh_file>")
        sys.exit(1)
    mesh_file = sys.argv[1]
    try:
        publish_mesh(mesh_file)
    except rospy.ROSInterruptException:
        pass
