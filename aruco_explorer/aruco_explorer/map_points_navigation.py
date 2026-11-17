import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from std_msgs.msg import Bool, Int32MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


SIGNAL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

class MapPointsNavigation(Node):

    def __init__(self):
        super().__init__('map_points_navigation')

        self.waypoints = []
        self.start_requested = False
        self.cancel_requested = False

        self.subscription = self.create_subscription(
            MarkerArray,
            '/map_points',
            self.points_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/exploration_start',
            self.start_callback,
            SIGNAL_QOS
        )

        self.create_subscription(
            Bool,
            '/exploration_cancel',
            self.cancel_callback,
            SIGNAL_QOS
        )

        self.get_logger().info('Waiting for map points and action goal...')

    def start_callback(self, msg):
        if msg.data: self.start_requested = True

    def cancel_callback(self, msg):
        if msg.data: self.cancel_requested = True

    def points_callback(self, msg):
        if self.waypoints: return

        markers = [
            marker for marker in msg.markers
            if marker.ns == 'map_points' and marker.type == Marker.SPHERE
        ]

        if not markers: return

        markers.sort(key=lambda marker: marker.id)

        for marker in markers:
            self.waypoints.append((marker.pose.position.x, marker.pose.position.y, marker.header.frame_id or 'map'))

        self.get_logger().info(f'Get {len(self.waypoints)} points.')


def wait_for_task(navigator, timeout_sec):
    start = time.monotonic()

    while rclpy.ok() and not navigator.isTaskComplete():
        if navigator.cancel_requested:
            navigator.cancelTask()
            return TaskResult.CANCELED
        
        if time.monotonic() - start > timeout_sec:
            navigator.get_logger().warn('Time limit exceed. Cancel task.')
            navigator.cancelTask()


            cancel_start = time.monotonic()
            while (rclpy.ok() and not navigator.isTaskComplete() and time.monotonic() - cancel_start < 10):
                time.sleep(0.05)

            return TaskResult.CANCELED

    return navigator.getResult()


def main(args=None):
    rclpy.init(args=args)

    receiver = MapPointsNavigation()
    navigator = None

    try:
        # while rclpy.ok() and not receiver.waypoints: rclpy.spin_once(receiver, timeout_sec=0.5)
        # if not rclpy.ok(): return

        while rclpy.ok() and not receiver.cancel_requested and (not receiver.waypoints or not receiver.start_requested): rclpy.spin_once(receiver, timeout_sec=0.5)
        if not rclpy.ok() or receiver.cancel_requested: return

        waypoints = list(receiver.waypoints)
        receiver.destroy_node()
        receiver = None

        navigator = BasicNavigator()
        navigator.cancel_requested = False

        def on_cancel(msg):
            if msg.data: navigator.cancel_requested = True

        navigator.create_subscription(
            Bool,
            '/exploration_cancel',
            on_cancel,
            SIGNAL_QOS
        )

        finished_pub = navigator.create_publisher(
            Bool,
            '/exploration_finished',
            SIGNAL_QOS
        )

        progress_pub = navigator.create_publisher(
            Int32MultiArray,
            '/exploration_progress',
            10
        )

        navigator.get_logger().info('Waiting for Nav2...')
        navigator.waitUntilNav2Active()

        total = len(waypoints)
        progress_pub.publish(Int32MultiArray(data=[0, total]))


        for i, (x, y, frame) in enumerate(waypoints, start=1):
            if navigator.cancel_requested: break

            navigator.get_logger().info(f'Going to P{i}: x={x:.2f}, y={y:.2f}')

            goal = PoseStamped()
            goal.header.frame_id = frame
            goal.header.stamp = navigator.get_clock().now().to_msg()

            goal.pose.position.x = x
            goal.pose.position.y = y

            goal.pose.orientation.w = 1.0

            navigator.goToPose(goal)

            result = wait_for_task(navigator, timeout_sec=120)

            if navigator.cancel_requested: break

            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(f'Reached P{i}. Spinning.')

                navigator.spin(spin_dist=2 * math.pi, time_allowance=60)
                spin_result = wait_for_task(navigator, timeout_sec=70)

                if navigator.cancel_requested: break
                if spin_result != TaskResult.SUCCEEDED: navigator.get_logger().warn(f'Spin failed at P{i}.')

            else:
                navigator.get_logger().warn(f'Could not get to P{i}.')

            progress_pub.publish(Int32MultiArray(data=[i, total]))
            navigator.get_logger().info(f'')


        if not navigator.cancel_requested:
            navigator.get_logger().info('All points processed.')
            finished_pub.publish(Bool(data=True))
            time.sleep(0.5)
        else:
            navigator.get_logger().warn('Exploration canceled by action client.')

    except KeyboardInterrupt:
        if navigator is not None and rclpy.ok():
            navigator.cancelTask()

    finally:
        if receiver is not None:
            receiver.destroy_node()

        if navigator is not None:
            navigator.destroyNode()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
