import rclpy
from rclpy.node import Node
import numpy as np
import pyproj
import pickle
import os

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

try:
    from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
except ImportError:
    pass

class AdaptiveTramOdometry(Node):
    def __init__(self):
        super().__init__('adaptive_tram_odometry')

        self.dt = 0.05
        
        # Kalman Filter State: [distance (m), velocity (m/s)]
        self.x = np.array([[0.0], [0.0]])
        self.P = np.eye(2) * 1.0
        
        self.F = np.array([[1.0, self.dt], [0.0, 1.0]])
        self.H = np.array([[0.0, 1.0]])
        self.Q_base = np.array([[0.01, 0.0], [0.0, 0.1]])
        self.R_base = np.array([[0.1]])
        
        self.last_time = None
        self.current_cmd = 0
        self.v_front = 0.0
        self.v_rear = 0.0
        
        # Map Matching State
        self.proj = pyproj.Proj(proj='utm', zone=37, ellps='WGS84')
        self.gnss_buffer = []
        self.map_locked = False
        self.current_path = None
        self.path_idx = 0
        self.current_pos = np.array([0.0, 0.0])
        self.heading = 0.0
        self.last_distance = 0.0
        
        # Load Offline Map
        self.load_map()
        
        # Subscribers
        self.sub_v_front = self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity', self.v_front_cb, 10)
        self.sub_v_rear = self.create_subscription(VelocitySensor, '/vehicle/rear_bogie_velocity', self.v_rear_cb, 10)
        self.sub_cmd = self.create_subscription(DriverControllerCommand, '/vehicle/driver_position_cmd', self.cmd_cb, 10)
        self.sub_gnss_fix = self.create_subscription(NavSatFix, '/sensing/gnss/master/fix', self.gnss_fix_cb, 10)
            
        # Publishers
        self.pub_vel = self.create_publisher(VelocitySensor, '/result/velocity', 10)
        self.pub_odom = self.create_publisher(Odometry, '/result/position', 10)

        self.get_logger().info("Adaptive Tram Odometry Node with Map Matching started.")

    def load_map(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('tram_odometry_pkg')
            map_path = os.path.join(pkg_dir, 'tram_map.pkl')
        except Exception:
            # Fallback for local testing if not built with colcon
            map_path = 'tram_map.pkl'
            
        if os.path.exists(map_path):
            with open(map_path, 'rb') as f:
                self.offline_paths = pickle.load(f)
            self.get_logger().info(f"Loaded {len(self.offline_paths)} paths from map ({map_path}).")
        else:
            self.offline_paths = []
            self.get_logger().warning(f"Map {map_path} not found! Will fallback to straight-line dead reckoning.")

    def gnss_fix_cb(self, msg):
        if not self.map_locked:
            x, y = self.proj(msg.longitude, msg.latitude)
            self.gnss_buffer.append(np.array([x, y]))
            
            # Once we have 5 points, we lock onto the map
            if len(self.gnss_buffer) >= 5:
                self.lock_to_map()

    def lock_to_map(self):
        if len(self.offline_paths) == 0:
            self.map_locked = True
            self.current_pos = self.gnss_buffer[0]
            # Estimate heading from GNSS buffer
            d = self.gnss_buffer[-1] - self.gnss_buffer[0]
            self.heading = np.arctan2(d[1], d[0])
            self.get_logger().info("Fallback to straight-line. No map available.")
            return

        # Find the path that best matches our initial GNSS points
        best_path = None
        best_idx = 0
        min_error = float('inf')
        
        start_pt = self.gnss_buffer[0]
        end_pt = self.gnss_buffer[-1]
        
        for path in self.offline_paths:
            # Find closest point on this path to our start_pt
            dists = np.linalg.norm(path - start_pt, axis=1)
            idx = np.argmin(dists)
            
            # Check direction match if path is long enough
            if idx + 5 < len(path):
                path_dir = path[idx + 5] - path[idx]
                gnss_dir = end_pt - start_pt
                
                # Normalize
                n_path = np.linalg.norm(path_dir)
                n_gnss = np.linalg.norm(gnss_dir)
                if n_path > 0 and n_gnss > 0:
                    dot = np.dot(path_dir/n_path, gnss_dir/n_gnss)
                    # If heading roughly matches and it's close
                    if dot > 0.8 and dists[idx] < min_error:
                        min_error = dists[idx]
                        best_path = path
                        best_idx = idx

        if best_path is not None:
            self.current_path = best_path
            self.path_idx = best_idx
            self.current_pos = best_path[best_idx]
            self.get_logger().info(f"Locked onto map! Error: {min_error:.2f}m")
        else:
            self.current_pos = self.gnss_buffer[0]
            self.get_logger().warning("Could not lock to any path, falling back.")
            
        self.map_locked = True

    def v_front_cb(self, msg):
        self.v_front = msg.velocity
        self.update_filter(msg.header.stamp)

    def v_rear_cb(self, msg):
        self.v_rear = msg.velocity

    def cmd_cb(self, msg):
        self.current_cmd = msg.position

    def get_time_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def update_filter(self, stamp):
        if not self.map_locked:
            return # Wait for initial GNSS
            
        current_time = self.get_time_sec(stamp)
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = current_time - self.last_time
        if dt <= 0: return
        self.last_time = current_time
        
        # 1. Prediction
        self.F[0, 1] = dt
        accel_cmd = (self.current_cmd / 15.0) * 1.5 
        B = np.array([[0.5 * dt**2], [dt]])
        u = np.array([[accel_cmd]])
        
        x_pred = self.F @ self.x + B @ u
        Q = self.Q_base * (1.0 + abs(self.current_cmd) * 0.5)
        P_pred = self.F @ self.P @ self.F.T + Q
        
        # 2. Update
        z = np.array([[(self.v_front + self.v_rear) / 2.0]])
        R = self.R_base * (1.0 + (abs(self.current_cmd) ** 2) * 10.0)
        
        if abs(self.v_front - self.v_rear) > 0.5:
            R *= 100.0
            
        y = z - (self.H @ x_pred)
        S = self.H @ P_pred @ self.H.T + R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        
        self.x = x_pred + K @ y
        self.P = (np.eye(2) - K @ self.H) @ P_pred
        
        # 3. Map traverse
        current_distance = float(self.x[0, 0])
        ds = current_distance - self.last_distance
        self.last_distance = current_distance
        
        self.traverse_map(ds)
        self.publish_results(stamp)

    def traverse_map(self, ds):
        if self.current_path is None:
            # Fallback
            self.current_pos[0] += ds * np.cos(self.heading)
            self.current_pos[1] += ds * np.sin(self.heading)
            return
            
        # Move ds along the path
        remaining = ds
        while remaining > 0 and self.path_idx < len(self.current_path) - 1:
            p1 = self.current_pos
            p2 = self.current_path[self.path_idx + 1]
            seg_len = np.linalg.norm(p2 - p1)
            
            if remaining < seg_len:
                # Move partially along segment
                direction = (p2 - p1) / seg_len
                self.current_pos = p1 + direction * remaining
                remaining = 0
            else:
                # Snap to next point and reduce remaining
                self.current_pos = p2
                self.path_idx += 1
                remaining -= seg_len

    def publish_results(self, stamp):
        v_msg = VelocitySensor()
        v_msg.header.stamp = stamp
        v_msg.header.frame_id = 'base_link'
        v_msg.velocity = float(self.x[1, 0])
        self.pub_vel.publish(v_msg)
        
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = float(self.current_pos[0])
        odom_msg.pose.pose.position.y = float(self.current_pos[1])
        odom_msg.pose.pose.position.z = 0.0
        
        self.pub_odom.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveTramOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
