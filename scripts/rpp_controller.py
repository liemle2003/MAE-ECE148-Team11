#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import scipy.interpolate as si

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.get_logger().info('='*50)
        self.get_logger().info('Pure Pursuit Controller Node Starting...')
        self.get_logger().info('Initializing parameters and subscribers...')
        self.get_logger().info('='*50)

    
    # Add initialization of position variables
        self.x = None
        self.y = None
        self.yaw = None
        
        # Controller parameters
        self.lookahead_distance = 0.15  # meters
        self.linear_velocity = 0.1      # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.goal_tolerance = 0.05      # meters

        # Initialize path tracking variables
        self.current_path_index = 0
        self.interpolated_path = None
        self.robot_pose = None

        # Set up TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.path_sub = self.create_subscription(Path, 'rrt_path_topic', self.path_callback, 10)
        #self.position_sub = self.create_subscription(PoseStamped, '/robot_position', self.position_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.position_callback, 10)

        # Control loop timer (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller initialized.')
    
    # Add diagnostic timer
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_check)

    def diagnostic_check(self):
        """Periodic diagnostic check of vital data"""
        if self.x is None or self.y is None:
            self.get_logger().warn("No position data received yet!")
            # Check if odometry topic is being published
            self.get_logger().info("Checking if odom topic is active...")
        else:
            self.get_logger().info(f"Position data OK: x={self.x:.4f}, y={self.y:.4f}")

        # Check if path data has been received
        if self.interpolated_path is None:
            self.get_logger().warn("No path data received yet!")

        

    def position_callback(self, msg):
        """Receive robot's position."""
        try:
        # Directly assign values without creating tuples
            self.x = float(msg.pose.pose.position.x)  # Convert to float and remove comma
            self.y = float(msg.pose.pose.position.y)  # Convert to float and remove comma
            self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
            self.get_logger().info(f"Robot Pose Updated: x={self.x:.4f}, y={self.y:.4f}, yaw={self.yaw:.4f}")
        except Exception as e:
            self.get_logger().error(f"Error processing position update: {str(e)}")

    def path_callback(self, msg):
        """Receive and process the path."""
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")
        if not msg.poses:
            self.get_logger().warn('Received empty path.')
            self.stop_robot()
            return

        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.interpolated_path = self.bspline_planning(path_points, len(path_points) * 5)

        if self.interpolated_path:
            self.current_path_index = 0
            self.get_logger().info(f'Received path with {len(self.interpolated_path)} waypoints.')
        else:
            self.get_logger().warn("Failed to process the path.")
            self.stop_robot()

    def bspline_planning(self, points, sn):
        """Smooth the path using B-spline interpolation."""
        try:
            if len(points) < 2:
                self.get_logger().warn("Not enough points for B-spline interpolation.")
                return points

            array = np.array(points)
            x, y = array[:, 0], array[:, 1]
            N = 2  # Degree of B-spline

            t = range(len(x))
            x_tup = si.splrep(t, x, k=N)
            y_tup = si.splrep(t, y, k=N)

            ipl_t = np.linspace(0.0, len(x) - 1, sn)
            rx = si.splev(ipl_t, x_tup)
            ry = si.splev(ipl_t, y_tup)

            return list(zip(rx, ry))
        except Exception as e:
            self.get_logger().warn(f"B-spline interpolation failed: {e}")
            return points

    def control_loop(self):
        """Main control loop for pure pursuit."""
        if self.interpolated_path is None: #or self.robot_pose is None:
            self.stop_robot()
            return

        # Check if the goal has been reached
        if self.is_goal_reached():
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            self.interpolated_path = None
            return

        # Find the next target point
        target_point = self.find_lookahead_point()
        if target_point is not None:
            self.get_logger().info(
                    f"Lookahead Point: {target_point}, Current Pose: ({self.x}, {self.y})"
            )
        else:
            self.get_logger().warn("No valid lookahead point found.")
            self.stop_robot()
            return

        # Publish velocity command
        self.publish_cmd_vel(target_point)

    def publish_cmd_vel(self, target_point):
        """Calculate and publish velocity commands."""
        if target_point is None:
            self.get_logger().warn("No target point found. Stopping the robot.")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        if target_point is not None:
            self.get_logger().info(f"Publishing velocity command. Target: {target_point}")

        target_x, target_y = target_point
        robot_x, robot_y, robot_yaw = self.x, self.y, self.yaw

        target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = target_angle - robot_yaw

        # Normalize heading error to range [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        
        if self.current_path_index == 0 and abs(heading_error) > math.pi/6:
            self.get_logger().info("Aligning with path...")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = math.copysign(self.max_angular_velocity, heading_error)  # Turn in place
            self.cmd_vel_pub.publish(cmd_vel)
            return

        cmd_vel = Twist()
        if abs(heading_error) > math.pi / 4:  # Large heading error: Turn in place
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = math.copysign(self.max_angular_velocity, heading_error)
            self.get_logger().info(f"Turning in place. Heading error: {heading_error:.2f}")
        else:  # Small heading error: Move forward and turn
            cmd_vel.linear.x = self.linear_velocity
            cmd_vel.angular.z = self.calculate_angular_velocity(heading_error)
            self.get_logger().info(f"Moving forward. Heading error: {heading_error:.2f}")

        self.cmd_vel_pub.publish(cmd_vel)

    def find_lookahead_point(self):
        """Find the next waypoint for the robot."""
        robot_x, robot_y = self.x, self.y
        for i in range(self.current_path_index, len(self.interpolated_path)):
            point_x, point_y = self.interpolated_path[i]
            distance = math.sqrt((point_x - robot_x) ** 2 + (point_y - robot_y) ** 2)
            if distance >= self.lookahead_distance:
                self.current_path_index = i
                return self.interpolated_path[i]
        return None

    def calculate_angular_velocity(self, heading_error):
        """Calculate angular velocity using proportional control."""
        kp = 1.0
        return np.clip(kp * heading_error, -self.max_angular_velocity, self.max_angular_velocity)

    def is_goal_reached(self):
        """Check if the robot has reached the goal."""
        goal_x, goal_y = self.interpolated_path[-1]
        robot_x, robot_y = self.x, self.y
        return math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2) < self.goal_tolerance

    def stop_robot(self):
        """Safely stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Controller interrupted by user.")
    finally:
        if rclpy.ok():
            controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

