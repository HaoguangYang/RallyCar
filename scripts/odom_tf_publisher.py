#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler angles -- roll (x), pitch (y), yaw (z), angles are in radians,
    into a Quaternion [qx, qy, qz, qw], returns a list of the four numbers.
    """
    ehalf = np.array([roll, pitch, yaw]) * 0.5
    ci, cj, ck = np.cos(ehalf)
    si, sj, sk = np.sin(ehalf)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    # return q in [x, y, z, w] order
    return [cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss]


class OdomTfPublisher(Node):
    """
    Converts an updating Odometry message into a tf pointing to a specified
    baselink frame. A typical use case of this node is a helper to bridge a
    missing tf from odom to base_link. To achieve this functionality, the user
    should set the "updater_topic" parameter to the topic with a publisher of
    Odometry message. On the publisher side, the user should set the
    header.frame_id field of the Odometry message as "odom", and the pose
    field of the message being the integrated robot pose since it started moving.

    This node Subscribes to a specified topic with message type Odometry, and
    extracts the header, pose.position, and pose.orientation fields. This node
    then packs these fields into a TransformStamped message type and publishes
    to /tf.

    The user also needs to specify the initial values of the transform, if not
    identity.

    A guard of minium update frequency is set, such that when the update topic
    is not specified or not updating, old values are packed with a current
    timestamp and re-published at that guard frequency.
    """
    def __init__(self):
        super().__init__('odom_tf_publisher_node')

        # Declare and acquire `odom_frame_name` parameter
        self.declare_parameter('init_source_frame_name', 'odom')
        self.declare_parameter('target_frame_name', 'base_link')
        self.declare_parameter('init_tf_pose', [0., 0., 0., 0., 0., 0.])

        # build the initial value
        self.tf = TransformStamped()
        self.tf.header.frame_id = self.get_parameter('init_source_frame_name').value
        self.tf.child_frame_id = self.get_parameter('target_frame_name').value
        tf_init = self.get_parameter('init_tf_pose').value
        # unpack the first three values into translation
        t = self.tf.transform.translation
        t.x, t.y, t.z = tf_init[0:3]
        # unpack the last three values into rotation (convert to quaternion)
        q = self.tf.transform.rotation
        q.x, q.y, q.z, q.w = quaternion_from_euler(tf_init[3], tf_init[4], tf_init[5])

        # User may specify a topic where this node receives updates.
        # if this parameter is non-empty, a subscriber of type
        # 'nav_msgs/Odometry' is created at that topic.
        # This node directly picks the header, pose.position, and pose.orientation,
        # fields, and updates the published transform.
        self.declare_parameter('updater_topic', '')
        sub_topic = self.get_parameter('updater_topic').value
        self.subscription = None
        if sub_topic:
            # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
            # callback function on each message
            self.subscription = self.create_subscription(
                Odometry,
                sub_topic,
                self.handle_tf_update,
                rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
            )

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # set a timer callback to guard the minimum update frequency.
        self.declare_parameter('min_tf_broadcast_frequency', 10.0)
        tf_freq = self.get_parameter('min_tf_broadcast_frequency').value
        self.send_tf_timer = self.create_timer(1./tf_freq, self.send_tf_callback)


    def send_tf_callback(self):
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf)


    def handle_tf_update(self, msg):
        self.tf.header = msg.header
        self.tf.child_frame_id = msg.child_frame_id

        # translate the field names
        pos = msg.pose.pose.position
        t = self.tf.transform.translation
        t.x, t.y, t.z = (pos.x, pos.y, pos.z)
        self.tf.transform.rotation = msg.pose.pose.orientation

        # we just received an update and have published, therefore we postpone
        # the next time-driven update by resetting the timer.
        self.send_tf_timer.reset()
        # sending the translated tf
        self.tf_broadcaster.sendTransform(self.tf)


def main():
    rclpy.init()
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
