#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast_sensor_tf():
    rospy.init_node('sensor_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        # Replace these values with your desired position and orientation
        translation = (1.0, 1.0, 1.0)  # x, y, z
        rotation = tf.transformations.quaternion_from_euler(0, 0, 0)  # roll, pitch, yaw

        br.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "sensor_link",
            "base_link"
        )

        rate.sleep()

if __name__ == "__main__":
    try:
        broadcast_sensor_tf()
    except rospy.ROSInterruptException:
        pass
