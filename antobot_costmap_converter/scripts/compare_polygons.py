#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from costmap_converter_msgs.msg import ObstacleArrayMsg


class ComparePolygons(Node):
    def __init__(self):
        super().__init__('compare_polygons')
        self.get_logger().info('compare_polygons started')
        self.sub_plugin = self.create_subscription(
            ObstacleArrayMsg, '/costmap_converter/obstacles', self.cb_plugin, 10)
        self.sub_custom = self.create_subscription(
            ObstacleArrayMsg, '/costmap_converter/obstacles_custom', self.cb_custom, 10)
        self.last_plugin = None
        self.last_custom = None
        self.timer = self.create_timer(0.5, self.tick)

    def cb_plugin(self, msg: ObstacleArrayMsg):
        self.last_plugin = msg

    def cb_custom(self, msg: ObstacleArrayMsg):
        self.last_custom = msg

    def tick(self):
        if self.last_plugin is None or self.last_custom is None:
            self.get_logger().info('waiting for both topics...')
            return
        p = self.last_plugin
        c = self.last_custom
        same = self.equal_obstacles(p, c)
        self.get_logger().info(f'Equal: {same} | plugin={len(p.obstacles)} custom={len(c.obstacles)}')

    def equal_obstacles(self, a: ObstacleArrayMsg, b: ObstacleArrayMsg) -> bool:
        if len(a.obstacles) != len(b.obstacles):
            return False
        def poly_to_tuple_list(poly):
            return [(round(pt.x, 3), round(pt.y, 3)) for pt in poly.points]
        def normalize(poly):
            pts = poly_to_tuple_list(poly)
            if not pts:
                return []
            # rotate to minimal lexicographic start
            m = min(range(len(pts)), key=lambda i: pts[i])
            rot = pts[m:] + pts[:m]
            # if reversed is smaller, use it
            rev = list(reversed(rot))
            return min(rot, rev)
        A = [normalize(o.polygon) for o in a.obstacles]
        B = [normalize(o.polygon) for o in b.obstacles]
        A.sort()
        B.sort()
        return A == B


def main():
    rclpy.init()
    node = ComparePolygons()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
