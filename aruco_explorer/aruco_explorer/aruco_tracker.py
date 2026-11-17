import time
from threading import Event, Lock

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from aruco_explorer_interfaces.action import FindAruco
from std_msgs.msg import Int32MultiArray

import csv
from collections import defaultdict, deque
from math import hypot, floor
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from aruco_msgs.msg import MarkerArray as ArucoArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


SIGNAL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        self.raw = defaultdict(lambda: deque(maxlen=5000))
        self.views = defaultdict(dict)

        self.data_lock = Lock()
        self.state_lock = Lock()
        self.finished_event = Event()

        self.active = False
        self.busy = False
        self.completed = False
        self.stopping = False
        self.shutdown_at = None

        self.visited_points = 0
        self.total_points = 0

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        self.file = open('/arm_ws/aruco_positions.csv', 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['time', 'id', 'x', 'y', 'z', 'robot_x', 'robot_y'])

        self.create_subscription(
            ArucoArray,
            '/marker_publisher/markers',
            self.receive,
            10
        )

        self.create_subscription(
            Bool,
            '/exploration_finished',
            self.on_finish,
            SIGNAL_QOS
        )

        self.create_subscription(
            Int32MultiArray,
            '/exploration_progress',
            self.on_progress,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            '/aruco/map_markers',
            10
        )

        self.create_timer(1.0, self.publish)

        self.start_pub = self.create_publisher(
            Bool,
            '/exploration_start',
            SIGNAL_QOS
        )

        self.cancel_pub = self.create_publisher(
            Bool,
            '/exploration_cancel',
            SIGNAL_QOS
        )

        self.action_server = ActionServer(
            self,
            FindAruco,
            '/find_aruco',
            execute_callback=self.execute_action,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('Action server /find_aruco ready.')

    def receive(self, msg):
        if not self.active: return

        try:
            tr = self.tf.lookup_transform('map', 'base_link', Time()).transform.translation
            robot = (tr.x, tr.y)

        except TransformException:
            robot = None

        with self.data_lock:
            if not self.active: return

            for tag in msg.markers:
                frame = tag.header.frame_id or msg.header.frame_id
                if frame != 'map': continue

                tag_id = int(tag.id)
                p = tag.pose.pose.position
                xyz = (p.x, p.y, p.z)

                stamp = tag.header.stamp
                if not (stamp.sec or stamp.nanosec): stamp = msg.header.stamp

                t = stamp.sec + stamp.nanosec * 1e-9
                self.csv.writerow([t, tag_id, *xyz, *(robot or ('', ''))])
                self.raw[tag_id].append(Point(x=p.x, y=p.y, z=p.z))

                if robot is not None:
                    cell = (floor(robot[0] / 0.5), floor(robot[1] / 0.5))
                    self.views[tag_id].setdefault(cell, deque(maxlen=30)).append(xyz)

            self.file.flush()

    def on_goal(self, goal_request):
        with self.state_lock:
            if not goal_request.start or self.busy or self.completed:
                return GoalResponse.REJECT

            self.busy = True

        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def on_progress(self, msg):
        if len(msg.data) >= 2 and self.active:
            with self.data_lock:
                self.visited_points = int(msg.data[0])
                self.total_points = int(msg.data[1])

    def on_finish(self, msg):
        if msg.data and self.active:
            self.get_logger().info('Navigation finished.')
            self.finished_event.set()

    def save_final(self):
        positions = []

        with self.data_lock:
            self.file.flush()

            with open('/arm_ws/aruco_positions.txt', 'w') as file:
                for tag_id in sorted(self.raw):
                    best = self.best_position(tag_id)

                    if best is None: continue

                    file.write(
                        f'Aruco {tag_id}: '
                        f'x: {best[0]} y: {best[1]}\n'
                    )

                    positions.append((tag_id,Point(x=best[0], y=best[1], z=best[2])))

        self.get_logger().info('Saved aruco_positions.txt')

        return positions

    def execute_action(self, goal_handle):
        with self.data_lock:
            self.raw.clear()
            self.views.clear()
            self.visited_points = 0
            self.total_points = 0

            self.file.seek(0)
            self.file.truncate()
            self.csv.writerow(['time', 'id', 'x', 'y', 'z', 'robot_x', 'robot_y'])
            self.file.flush()

            self.finished_event.clear()
            self.active = True

        self.cancel_pub.publish(Bool(data=False))
        self.start_pub.publish(Bool(data=True))

        self.get_logger().info('Goal accepted. Exploration started.')
        result = FindAruco.Result()

        try:
            while rclpy.ok() and not self.finished_event.wait(timeout=1.0):
                if goal_handle.is_cancel_requested or self.stopping:
                    self.cancel_pub.publish(Bool(data=True))

                    with self.data_lock: self.active = False

                    positions = self.save_final()

                    result.ids = [i for i, _ in positions]
                    result.positions = [p for _, p in positions]

                    if goal_handle.is_cancel_requested: goal_handle.canceled()
                    else: goal_handle.abort()

                    return result

                with self.data_lock:
                    feedback = FindAruco.Feedback()
                    feedback.visited_points = self.visited_points
                    feedback.total_points = self.total_points
                    feedback.found_tags = len(self.raw)

                goal_handle.publish_feedback(feedback)

            with self.data_lock: self.active = False

            positions = self.save_final()

            result.ids = [i for i, _ in positions]
            result.positions = [p for _, p in positions]

            if self.stopping or not rclpy.ok(): goal_handle.abort()
            else: goal_handle.succeed()

            return result

        finally:
            self.completed = True
            self.shutdown_at = time.monotonic() + 3.0

    def best_position(self, tag_id):
        candidates = [
            tuple(median(p[i] for p in samples) for i in range(3))
            for samples in self.views[tag_id].values()
            if len(samples) >= 3
        ]

        if candidates:
            return min(
                candidates,
                key=lambda a: sum(hypot(a[0] - b[0], a[1] - b[1]) for b in candidates)
            )

        history = self.raw[tag_id]

        if history:
            return (
                median(p.x for p in history),
                median(p.y for p in history),
                median(p.z for p in history),
            )

        return None

    def make_marker(self, tag_id, ns, kind, pos=(0., 0., 0.)):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = tag_id
        m.type = kind
        m.action = Marker.ADD

        m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])

        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        return m

    def publish(self):
        output = MarkerArray()

        with self.data_lock:
            for tag_id, history in self.raw.items():
                dots = self.make_marker(tag_id, 'raw', Marker.POINTS)

                dots.points = list(history)
                dots.scale.x = dots.scale.y = 0.035
                dots.color.b = 1.0
                dots.color.g = 0.5
                dots.color.a = 0.6
                output.markers.append(dots)

                best = self.best_position(tag_id)

                if best is None: continue

                ball = self.make_marker(tag_id, 'best', Marker.SPHERE, best)

                ball.scale.x = ball.scale.y = ball.scale.z = 0.14
                ball.color.r = 1.0
                output.markers.append(ball)

                label = self.make_marker(tag_id, 'labels', Marker.TEXT_VIEW_FACING, (best[0], best[1], best[2] + 0.25))

                label.scale.z = 0.2
                label.text = f'ArUco {tag_id}'
                output.markers.append(label)

        self.pub.publish(output)



def main(args=None):
    rclpy.init(args=args)

    node = ArucoTracker()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)
            if (node.shutdown_at is not None and time.monotonic() >= node.shutdown_at): break

    except KeyboardInterrupt:
        node.stopping = True
        node.cancel_pub.publish(Bool(data=True))
        node.finished_event.set()

        if not node.busy:
            node.save_final()

    finally:
        node.stopping = True
        node.finished_event.set()

        executor.shutdown()
        node.action_server.destroy()
        node.file.close()
        node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
