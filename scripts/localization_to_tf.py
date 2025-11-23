#!/usr/bin/env python3
"""
localization_to_tf.py

Subscribe to a pose/odom topic (selected at runtime by inspecting topic types)
and broadcast it as a tf: parent_frame -> child_frame.

Usage (example):
  python3 localization_to_tf.py --topic /localization/pose --frame map --child base_link

Supports these topic message types:
 - geometry_msgs/msg/PoseStamped
 - geometry_msgs/msg/PoseWithCovarianceStamped
 - nav_msgs/msg/Odometry
"""
import rclpy
from rclpy.node import Node
import argparse
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

TYPE_MAP = {
    'geometry_msgs/msg/PoseStamped': PoseStamped,
    'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
    'nav_msgs/msg/Odometry': Odometry,
}

class PoseToTf(Node):
    def __init__(self, topic_name: str, parent_frame: str, child_frame: str):
        super().__init__('pose_to_tf_broadcaster')
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.br = TransformBroadcaster(self)

        # find actual topic type(s)
        topic_types = self._find_topic_types(topic_name)
        if not topic_types:
            self.get_logger().error(f"No topic named '{topic_name}' (or matching) found in the system.")
            raise RuntimeError("Topic not found")

        # pick the first supported type
        chosen_type = None
        chosen_msg_cls = None
        for t in topic_types:
            if t in TYPE_MAP:
                chosen_type = t
                chosen_msg_cls = TYPE_MAP[t]
                break

        if chosen_msg_cls is None:
            self.get_logger().error(
                f"Topic '{topic_name}' has types {topic_types} but none are supported. Supported: {list(TYPE_MAP.keys())}"
            )
            raise RuntimeError("Unsupported topic type")

        # create subscription for the detected type
        try:
            self.get_logger().info(f"Subscribing to '{topic_name}' with type '{chosen_type}'")
            # set up appropriate callback
            if chosen_type == 'geometry_msgs/msg/PoseStamped':
                self.sub = self.create_subscription(PoseStamped, topic_name, self.cb_pose_stamped, 10)
            elif chosen_type == 'geometry_msgs/msg/PoseWithCovarianceStamped':
                self.sub = self.create_subscription(PoseWithCovarianceStamped, topic_name, self.cb_pose_cov, 10)
            elif chosen_type == 'nav_msgs/msg/Odometry':
                self.sub = self.create_subscription(Odometry, topic_name, self.cb_odom, 10)
        except Exception as e:
            self.get_logger().error(f"Failed to create subscription: {e}")
            raise

        self.get_logger().info(f"Listening to {topic_name} and broadcasting {parent_frame} -> {child_frame}")

    def _find_topic_types(self, topic_name):
        """
        Return list of types for a topic. Try exact match, and also suffix match
        (to tolerate leading remappings like 'rt/<topic>').
        """
        all_topics = self.get_topic_names_and_types()
        # exact match
        for name, types in all_topics:
            if name == topic_name:
                return types
        # try match ignoring leading namespaces (endswith)
        matches = []
        for name, types in all_topics:
            if name.endswith(topic_name) or name.rstrip('/') == topic_name.lstrip('/'):
                matches.extend(types)
        # deduplicate preserving order
        seen = []
        for t in matches:
            if t not in seen:
                seen.append(t)
        return seen

    def _send_transform(self, trans: TransformStamped):
        trans.header.stamp = self.get_clock().now().to_msg()
        # ensure parent/child frames
        trans.header.frame_id = self.parent_frame
        trans.child_frame_id = self.child_frame
        self.br.sendTransform(trans)

    def cb_pose_stamped(self, msg: PoseStamped):
        t = TransformStamped()
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._send_transform(t)

    def cb_pose_cov(self, msg: PoseWithCovarianceStamped):
        t = TransformStamped()
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._send_transform(t)

    def cb_odom(self, msg: Odometry):
        t = TransformStamped()
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._send_transform(t)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/localization/pose',
                        help='Pose topic to subscribe (PoseStamped / PoseWithCovarianceStamped / nav_msgs/Odometry)')
    parser.add_argument('--frame', default='map', help='Parent frame id (e.g. map)')
    parser.add_argument('--child', default='base_link', help='Child frame id (e.g. base_link)')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = PoseToTf(args.topic, args.frame, args.child)
    except Exception as e:
        # already logged inside node
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
