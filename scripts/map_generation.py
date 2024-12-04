#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from tf2_ros import TransformListener, Buffer
import numpy as np
import matplotlib.pyplot as plt

class GlobalMapNode(Node):
    def __init__(self):
        super().__init__('global_map_node')
        
        # Fixed parameters
        self.width = 200
        self.height = 200
        self.resolution = 0.05
        self.obstacle_radius = 2.0  # 2 meters
        
        # Initialize positions
        self.origin_x = None
        self.origin_y = None
        self.robot_x = None
        self.robot_y = None
        
        # Set fixed goal at top right corner (in meters)
        self.goal_x = 8.0  # Considering 200 * 0.05 = 10m total width
        self.goal_y = 8.0  # Setting slightly inside the bounds
        
        # Initialize empty grid
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        
        # ROS setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.previous_obstacles = set()

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/rrt_path_topic',
            self.path_callback,
            10
        )
        
        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/robot_position',
            10
        )
        
        self.rrt_service_client = self.create_client(
            SetBool,
            'trigger_rrt_replan'
        )
        
        # Setup visualization
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()
        
        self.get_logger().info('Global Map Node initialized')

    def map_callback(self, msg):
        """Handle incoming map data."""
        # Set origin if first time
        if self.origin_x is None:
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.get_logger().info(f'Map origin set to: ({self.origin_x}, {self.origin_y})')
            self.trigger_replan()  # Initial planning
        
        map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    #Only update cells that contain obstacle information
        #obstacle_indices = np.where(map_data == 100)
        #for i, j in zip(obstacle_indices[0], obstacle_indices[1]):
        #    if 0 <= i < self.height and 0 <= j < self.width:  # Ensure within bounds
        #        self.grid[i, j] = 100

        if abs(msg.info.resolution - self.resolution) > 1e-6:
            self.get_logger().error(
                f"Resolution mismatch! Expected: {self.resolution}, Got: {msg.info.resolution}"
            )
            return

        # Determine the region to update based on the smaller dimensions
        update_height = min(self.height, msg.info.height)
        update_width = min(self.width, msg.info.width)

        # Efficiently copy the relevant data into the fixed-size grid
        #self.grid[:update_height, :update_width] = map_data[:update_height, :update_width]
        obstacle_mask = map_data[:update_height, :update_width] == 100
        self.grid[:update_height, :update_width][obstacle_mask] = 100

        if not self.update_robot_position():
            self.get_logger().warn("Skipping obstacle checks: Robot position not ready")
            return
        
        # Update robot position and check for obstacles
        self.update_robot_position()
        self.publish_robot_position()
        self.check_for_obstacles_and_replan()
        self.update_visualization()

    def update_robot_position(self):
        """Get robot position from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return False
        return True
            #rclpy.sleep(rclpy.duration.Duration(seconds=0.5))
            #if self.robot_x is None or self.robot_y is None:
            #    self.robot_x = 0.0
            #    self.robot_y = 0.0

    def publish_robot_position(self):
        """Publish robot position as PoseStamped."""
        if self.robot_x is None or self.robot_y is None:
            return
            
        position_msg = PoseStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'map'
        position_msg.pose.position.x = self.robot_x
        position_msg.pose.position.y = self.robot_y
        position_msg.pose.position.z = 0.0
        
        self.position_publisher.publish(position_msg)
        self.get_logger().info(f"Published robot position: ({self.robot_x}, {self.robot_y})")

    def check_for_obstacles_and_replan(self):
        """Check if obstacles are within radius of robot."""
        if self.grid is None or self.robot_x is None:
            return
            
        # Convert robot position to grid indices
        robot_grid_x = int((self.robot_x - self.origin_x) / self.resolution)
        robot_grid_y = int((self.robot_y - self.origin_y) / self.resolution)
        
        # Define search radius in grid cells
        radius_in_cells = int(self.obstacle_radius / self.resolution)
        
        # Check for obstacles within radius
        current_obstacles = set()
        for i in range(max(0, robot_grid_y - radius_in_cells), 
                      min(self.height, robot_grid_y + radius_in_cells)):
            for j in range(max(0, robot_grid_x - radius_in_cells), 
                         min(self.width, robot_grid_x + radius_in_cells)):
                if self.grid[i, j] == 100:  # Obstacle detected
                    current_obstacles.add((i,j))
                    #self.get_logger().info("Obstacle detected nearby! Triggering RRT replanning.")

        new_obstacles = current_obstacles - self.previous_obstacles
        if new_obstacles:
            self.get_logger().info(f"New obstacles detected: {new_obstacles}. Triggering RRT replanning")
            self.previous_obstacles = current_obstacles
            self.trigger_replan()
        else:
            self.get_logger().info("No new obstacles detected. Skipped replanning")

    def trigger_replan(self):
        """Trigger RRT replanning service."""
        if not self.rrt_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Replan service not available')
            return False
        
        request = SetBool.Request()
        request.data = True
        
        try:
            future = self.rrt_service_client.call_async(request)
            self.get_logger().info('Replan service triggered')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to call replan service: {str(e)}')
            return False

    def path_callback(self, msg):
        """Handle incoming path updates."""
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) 
                            for pose in msg.poses]
        self.update_visualization()

    def update_visualization(self):
        """Update matplotlib visualization."""
        self.ax.clear()
        
        # Draw occupancy grid
        self.ax.imshow(
            self.grid,
            cmap='gray_r',
            origin='lower',
            extent=[
                self.origin_x if self.origin_x is not None else 0,
                self.origin_x + self.width * self.resolution if self.origin_x is not None else self.width * self.resolution,
                self.origin_y if self.origin_y is not None else 0,
                self.origin_y + self.height * self.resolution if self.origin_y is not None else self.height * self.resolution
            ]
        )
        
        # Draw robot position
        if self.robot_x is not None and self.robot_y is not None:
            self.ax.plot(self.robot_x, self.robot_y, 'bo', markersize=10, label='Robot')
            
            # Draw obstacle detection radius
            circle = plt.Circle(
                (self.robot_x, self.robot_y),
                self.obstacle_radius,
                fill=False,
                color='g',
                linestyle='--'
            )
            self.ax.add_artist(circle)
        
        # Draw goal
        self.ax.plot(self.goal_x, self.goal_y, 'yo', markersize=15, label='Goal')
        
        # Draw path if available
        if hasattr(self, 'current_path') and self.current_path:
            path_x, path_y = zip(*self.current_path)
            self.ax.plot(path_x, path_y, 'r-', label='RRT Path')
        
        self.ax.legend()
        self.ax.set_title("Global Map with RRT Path")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutting down...")
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()