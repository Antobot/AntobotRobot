#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_pub')
        self.declare_parameter('topic', '/costmap/footprint')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.pub = self.create_publisher(PolygonStamped, topic, 10)
        self.timer = self.create_timer(0.2, self.tick)
        # rectangle footprint around origin (~0.5m x 0.3m)
        self.points = [
            (0.25, 0.15),
            (0.25, -0.15),
            (-0.25, -0.15),
            (-0.25, 0.15),
            (0.25, 0.15),
        ]
        self.get_logger().info(f'publishing footprint on {topic}')

    def tick(self):
        msg = PolygonStamped()
        msg.header.frame_id = 'map'
        for x, y in self.points:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            msg.polygon.points.append(p)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FootprintPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

