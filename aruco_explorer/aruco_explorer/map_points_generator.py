import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class MapPointsGenerator(Node):

    def __init__(self):
        super().__init__('map_points_generator')

        self.declare_parameter('grid_spacing', 0.8)
        self.declare_parameter('max_cost', 20)
        self.declare_parameter('robot_start_x', 0.0)
        self.declare_parameter('robot_start_y', 0.0)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')

        self.points = []
        self.map_frame = 'map'
        self.generated = False
        self.clear_old_markers = True

        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('costmap_topic').value,
            self.costmap_callback,
            costmap_qos,
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/map_points', 10
                                                )
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Waiting for global Nav2 costmap...')

    def costmap_callback(self, msg):
        if self.generated: return

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        data = msg.data

        spacing = float(self.get_parameter('grid_spacing').value)
        max_cost = int(self.get_parameter('max_cost').value)
        start_x = float(self.get_parameter('robot_start_x').value)
        start_y = float(self.get_parameter('robot_start_y').value)

        if (width <= 0 or height <= 0 or resolution <= 0 or len(data) != width * height or spacing <= 0 or not (0 <= max_cost <= 98)):
            self.get_logger().error('Incorrect parameters or costmap.')
            return


        origin = msg.info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Robot start position
        dx = start_x - origin.position.x
        dy = start_y - origin.position.y
        seed_x = math.floor((cos_yaw * dx + sin_yaw * dy) / resolution)
        seed_y = math.floor((-sin_yaw * dx + cos_yaw * dy) / resolution)

        if not (0 <= seed_x < width and 0 <= seed_y < height):
            self.get_logger().error('Start position is out of the costmap.')
            return

        def passable(index):
            cost = data[index]
            return 0 <= cost <= max_cost

        seed = seed_y * width + seed_x

        if not passable(seed):
            self.get_logger().error(f'Start position has cost: {data[seed]}, max_cost={max_cost}')
            return


        # STEP 1. Floof Fill 
        reachable = bytearray(width * height)
        reachable[seed] = 1
        queue = deque([seed])

        while queue:
            index = queue.popleft()
            mx = index % width
            my = index // width

            for nx, ny in (
                (mx - 1, my),
                (mx + 1, my),
                (mx, my - 1),
                (mx, my + 1),
            ):
                if not (0 <= nx < width and 0 <= ny < height): continue

                neighbor = ny * width + nx

                if (not reachable[neighbor] and passable(neighbor)):
                    reachable[neighbor] = 1
                    queue.append(neighbor)

        # STEP 2: Map points
        step = max(1, round(spacing / resolution))
        offset = step // 2
        self.points.clear()

        for row_number, my in enumerate(range(offset, height, step)):
            x_cells = list(range(offset, width, step))

            if row_number % 2:
                x_cells.reverse()

            for mx in x_cells:
                index = my * width + mx

                if not reachable[index]: continue

                local_x = (mx + 0.5) * resolution
                local_y = (my + 0.5) * resolution

                x = origin.position.x + local_x * cos_yaw - local_y * sin_yaw
                y = origin.position.y + local_x * sin_yaw + local_y * cos_yaw
                self.points.append((x, y))

        self.map_frame = msg.header.frame_id or 'map'
        self.generated = True

        self.get_logger().info(
            f'Generated {len(self.points)} points. '
            f'Grid spacing: {step * resolution:.2f} m.'
        )

    def publish_markers(self):
        if not self.generated: return

        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()


        if self.clear_old_markers:
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.clear_old_markers = False

        for i, (x, y) in enumerate(self.points):
            point = Marker()
            point.header.frame_id = self.map_frame
            point.header.stamp = stamp
            point.ns = 'map_points'
            point.id = i
            point.type = Marker.SPHERE
            point.action = Marker.ADD

            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.10
            point.pose.orientation.w = 1.0

            point.scale.x = 0.20
            point.scale.y = 0.20
            point.scale.z = 0.10

            point.color.r = 0.0
            point.color.g = 1.0
            point.color.b = 0.2
            point.color.a = 1.0

            markers.markers.append(point)

            # Point number
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'point_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.40
            label.pose.orientation.w = 1.0

            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'P{i + 1}'

            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MapPointsGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
