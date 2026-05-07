import math
import os

import rclpy
from rclpy.node import Node

import numpy as np

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from marker_msgs.msg import MarkerDetection

from .map_parser import parse_world_file
from .grid_utils import wrap_angle_deg
from .filter import HistogramFilter
from . import config as cfg


class BayesFilterNode(Node):
    def __init__(self, world_path):
        super().__init__('bayes_filter')

        landmarks = parse_world_file(world_path)
        self.filter = HistogramFilter(landmarks)

        # Accumulated odom path
        self.initial_pose = cfg.INITIAL_POSE
        self.last_odom    = None

        self.odom_x = self.initial_pose[0]
        self.odom_y = self.initial_pose[1]
        self.odom_th = math.radians(self.initial_pose[2])

        # Path messages
        self.gt_path   = Path()
        self.gt_path.header.frame_id   = 'map'
        self.odom_path = Path()
        self.odom_path.header.frame_id = 'map'

        # Publishers
        self.costmap_pub  = self.create_publisher(OccupancyGrid, 'viz/belief_costmap', 10)
        self.landmark_pub = self.create_publisher(MarkerArray,   'viz/landmarks',       10)
        self.gt_pub       = self.create_publisher(Path,          'viz/gt_path',         10)
        self.odom_pub     = self.create_publisher(Path,          'viz/odom_path',       10)

        # Subscribers
        self.create_subscription(Odometry,         '/odom',         self._odom_cb,     10)
        self.create_subscription(Odometry,         '/ground_truth', self._gt_cb,       10)
        self.create_subscription(MarkerDetection,  '/fiducials',    self._fiducial_cb, 10)

        self.create_timer(1.0, self._publish_landmarks)

        self.get_logger().info('Bayes filter node started.')
        self.get_logger().info(f'Grid: {self.filter.grid_dim} x {self.filter.grid_dim} x {self.filter.theta_dim}')
        for fid, (fx, fy) in landmarks.items():
            self.get_logger().info(f'  Landmark {fid}: ({fx:.2f}, {fy:.2f})')

    def _log_best_estimate(self):
        ix, iy, ith = self.filter.best_estimate()

        x = cfg.WORLD_MIN + (ix + 0.5) * cfg.SPATIAL_RES
        y = cfg.WORLD_MAX - (iy + 0.5) * cfg.SPATIAL_RES
        theta = (ith + 0.5) * cfg.THETA_RES

        self.get_logger().info(
            f'Best belief: x={x:.2f}, y={y:.2f}, theta={theta:.1f} deg',
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _gt_cb(self, msg):
        p = PoseStamped()
        p.header.frame_id     = 'map'
        p.header.stamp        = msg.header.stamp
        p.pose.position.x     = msg.pose.pose.position.x
        p.pose.position.y     = msg.pose.pose.position.y
        p.pose.orientation.w  = 1.0

        self.gt_path.header.stamp = self.get_clock().now().to_msg()
        self.gt_path.poses.append(p)
        self.gt_path.poses = self.gt_path.poses[-200:]
        self.gt_pub.publish(self.gt_path)

    def _odom_cb(self, msg):
        if self.last_odom is None:
            self.last_odom = msg
            return

        x0  = self.last_odom.pose.pose.position.x
        y0  = self.last_odom.pose.pose.position.y
        th0 = self._yaw_deg(self.last_odom)

        x1  = msg.pose.pose.position.x
        y1  = msg.pose.pose.position.y
        th1 = self._yaw_deg(msg)

        dx      = x1 - x0
        dy      = y1 - y0
        d_trans = math.sqrt(dx * dx + dy * dy)
        d_theta = wrap_angle_deg(th1 - th0)

        if d_trans > 1e-6:
            direction = math.degrees(math.atan2(dy, dx))
            d_rot1 = wrap_angle_deg(direction - th0)
        else:
            d_rot1 = 0.0
        d_rot2 = wrap_angle_deg(d_theta - d_rot1)

        # Visualisation path — integrate odometry in the map frame
        self.odom_th += math.radians(d_theta)
        self.odom_x += (dx * math.cos(self.odom_th)) - (dy * math.sin(self.odom_th))
        self.odom_y += (dx * math.sin(self.odom_th)) + (dy * math.cos(self.odom_th))

        ox = self.odom_x
        oy = self.odom_y

        p = PoseStamped()
        p.header.frame_id    = 'map'
        p.header.stamp       = msg.header.stamp
        p.pose.position.x    = ox
        p.pose.position.y    = oy
        p.pose.orientation.w = 1.0

        self.odom_path.header.stamp = self.get_clock().now().to_msg()
        self.odom_path.poses.append(p)
        self.odom_path.poses = self.odom_path.poses[-200:]
        self.odom_pub.publish(self.odom_path)

        # Only run predict when there is meaningful motion
        if d_trans > 0.001 or abs(d_theta) > 0.1:
            self.filter.predict(d_rot1, d_trans, d_rot2)
            self.last_odom = msg
            self._publish_costmap()

    def _fiducial_cb(self, msg):
        for marker in msg.markers:
            result = self._parse_marker(marker)
            if result is None:
                continue
            fid, mrange, mbearing = result
            self.get_logger().info(
                f'Fiducial {fid}: range={mrange:.2f} m  bearing={mbearing:.1f} deg',
                throttle_duration_sec=1.0,
            )
            self.filter.update(fid, mrange, mbearing)
        self._publish_costmap()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _yaw_deg(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny, cosy))

    def _parse_marker(self, marker):
        marker_id = None
        if hasattr(marker, 'ids') and len(marker.ids) > 0:
            marker_id = int(marker.ids[0])
        else:
            for attr in ('id', 'fiducial_id', 'marker_id'):
                if hasattr(marker, attr):
                    marker_id = int(getattr(marker, attr))
                    break
        if marker_id is None:
            return None

        pos = None
        if hasattr(marker, 'pose'):
            pos = marker.pose.pose.position if hasattr(marker.pose, 'pose') else marker.pose.position
        elif hasattr(marker, 'position'):
            pos = marker.position
        if pos is None:
            return None

        mx, my = float(pos.x), float(pos.y)
        return marker_id, math.sqrt(mx*mx + my*my), math.degrees(math.atan2(my, mx))

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_costmap(self):
        belief_2d = np.sum(self.filter.belief, axis=2)
        # OccupancyGrid wants bottom row first; our grid row 0 is the top
        data = np.flipud(belief_2d.T)

        max_val = np.max(data)
        scaled  = (data / max_val * 100.0).astype(np.int8) if max_val > cfg.EPS else np.zeros_like(data, dtype=np.int8)

        grid = OccupancyGrid()
        grid.header.frame_id        = 'map'
        grid.header.stamp           = self.get_clock().now().to_msg()
        grid.info.resolution        = float(cfg.SPATIAL_RES)
        grid.info.width             = self.filter.grid_dim
        grid.info.height            = self.filter.grid_dim
        grid.info.origin.position.x = cfg.WORLD_MIN
        grid.info.origin.position.y = cfg.WORLD_MIN
        grid.info.origin.orientation.w = 1.0
        grid.data = scaled.flatten().tolist()
        self.costmap_pub.publish(grid)
        self._log_best_estimate()

    def _publish_landmarks(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for fid, (fx, fy) in self.filter.landmarks.items():
            cyl = Marker()
            cyl.header.frame_id = 'map'; cyl.header.stamp = now
            cyl.id = int(fid); cyl.type = Marker.CYLINDER; cyl.action = Marker.ADD
            cyl.pose.position.x = float(fx); cyl.pose.position.y = float(fy); cyl.pose.position.z = 0.5
            cyl.scale.x = cyl.scale.y = 0.3; cyl.scale.z = 1.0
            cyl.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            ma.markers.append(cyl)

            txt = Marker()
            txt.header.frame_id = 'map'; txt.header.stamp = now
            txt.id = int(fid) + 1000; txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
            txt.text = f'ID: {fid}'
            txt.pose.position.x = float(fx); txt.pose.position.y = float(fy) + 0.5; txt.pose.position.z = 1.2
            txt.scale.z = 0.4; txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            ma.markers.append(txt)

        self.landmark_pub.publish(ma)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    rclpy.init()

    world_path = os.path.expanduser('~/ros2_ws/src/stage_ros2/world/cave.world')
    if not os.path.exists(world_path):
        print(f'ERROR: world file not found: {world_path}')
        print('Run:  find ~/ros2_ws/src/stage_ros2 -name cave.world')
        return

    node = BayesFilterNode(world_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()