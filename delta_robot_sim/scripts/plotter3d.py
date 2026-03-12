#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformException


class DeltaEEPlotter(Node):
    def __init__(self):
        super().__init__("delta_ee_plotter")

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for the 3D Line
        self.marker_pub = self.create_publisher(Marker, "/delta_robot/ee_path", 10)

        # Initialize the Line Strip Marker
        self.marker = Marker()
        self.marker.header.frame_id = "delta_robot"
        self.marker.ns = "ee_trajectory"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD

        # Line appearance
        self.marker.scale.x = 0.005  # Line thickness (5mm)
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0  # Green line
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0  # Opacity

        # Timer to sample position at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up the transform
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                "delta_robot/world_link", "delta_robot/EE", now
            )

            # Extract translation
            p = Point()
            p.x = trans.transform.translation.x
            p.y = trans.transform.translation.y
            p.z = trans.transform.translation.z

            # Add to points list (keep last 500 points to prevent lag)
            self.marker.points.append(p)
            if len(self.marker.points) > 500:
                self.marker.points.pop(0)

            # Update timestamp and publish
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.marker)

        except TransformException as ex:
            self.get_logger().info(f"Could not transform: {ex}")


def main():
    rclpy.init()
    node = DeltaEEPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.init()


if __name__ == "__main__":
    main()
