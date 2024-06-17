#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates

def handle_model_states(msg):
    br = tf.TransformBroadcaster()
    
    try:
        idx = msg.name.index('structured_light_sensor_robot')
        
        # Extract the pose of the model
        pose = msg.pose[idx]
        
        br.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            rospy.Time.now(),
            "sensor_link",  # child link
            "base_link"     # parent link
        )
    except ValueError:
        pass

if __name__ == "__main__":
    rospy.init_node('gazebo_tf_publisher')
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_model_states)
    
    rospy.spin()
