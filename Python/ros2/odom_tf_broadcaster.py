#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry


def handle_odom(msg):
    """
    Callback function that triggers every time an /odom message arrives
    from the Arduino.
    """
    br = tf.TransformBroadcaster()

    # 1. Extract Position
    pos = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    )

    # 2. Extract Orientation (Quaternion)
    # The Arduino already calculated the quaternion, so we just pass it through.
    ori = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )

    # 3. Broadcast the Transform
    # Arguments: (translation, rotation, time, child_frame, parent_frame)
    br.sendTransform(
        pos,
        ori,
        msg.header.stamp,  # Use the timestamp from the Arduino message!
        "base_link",  # The robot's body
        "odom"  # The fixed frame where the robot started
    )


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('odom_tf_broadcaster')

    # Subscribe to the /odom topic published by rosserial
    rospy.Subscriber('/odom', Odometry, handle_odom)

    # Keep the node running
    rospy.spin()