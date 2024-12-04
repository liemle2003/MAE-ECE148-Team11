#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from ompl import base as ob
from ompl import geometric as og


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # Map properties with proper initialization
        self.width = 200  # Width in cells
        self.height = 200  # Height in cells
        self.resolution = 0.05  # 5 cm per cell
        self.origin_x = 0.0  # Initialize with default values
        self.origin_y = 0.0
        self.world_size_x = self.width * self.resolution  # Size in meters
        self.world_size_y = self.height * self.resolution

        # Initialize grid with proper dimensions
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_x = 8.0
        self.goal_y = 8.0
        
        # State flags
        self.grid_received = False
        self.position_received = False
        self.initial_plan_done = False

        # ROS subscriptions and publishers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/robot_position',
            self.position_callback,
            10
        )
        self.path_publisher = self.create_publisher(
            Path, 
            'rrt_path_topic', 
            10
        )
        self.service = self.create_service(
            SetBool,
            'trigger_rrt_replan',
            self.handle_replan_request
        )

        self.get_logger().info("RRT Planner Node initialized")

    def map_callback(self, msg):
        # Update origin from incoming map
        #self.origin_x = msg.info.origin.position.x
        #self.origin_y = msg.info.origin.position.y
        
        # Verify resolution match
        if abs(msg.info.resolution - self.resolution) > 1e-6:
            self.get_logger().error(
                f"Resolution mismatch! Expected: {self.resolution}, Got: {msg.info.resolution}"
            )
            return
        
        # Clear existing grid
        #self.grid.fill(0)
        
        # Update grid with new obstacles
        #for y in range(msg.info.height):
        #    for x in range(msg.info.width):
        #        if map_data[y, x] == 100:  # Obstacle
        #            world_x = self.origin_x + x * self.resolution
        #            world_y = self.origin_y + y * self.resolution
        #            grid_x, grid_y = self.world_to_grid(world_x, world_y)
        #            if grid_x is not None and grid_y is not None:
        #                self.grid[grid_y, grid_x] = 100
        if self.origin_x is None:
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.get_logger().info(f'Map origin set to: ({self.origin_x}, {self.origin_y})')
            #self.trigger_replan()  # Initial planning
        
        map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    #Only update cells that contain obstacle information
        #obstacle_indices = np.where(map_data == 100)
        #for i, j in zip(obstacle_indices[0], obstacle_indices[1]):
        #    if 0 <= i < self.height and 0 <= j < self.width:  # Ensure within bounds
        #        self.grid[i, j] = 100

        update_height = min(self.height, msg.info.height)
        update_width = min(self.width, msg.info.width)

        # Efficiently copy the relevant data into the fixed-size grid
        obstacle_mask = map_data[:update_height, :update_width] == 100
        self.grid[:update_height, :update_width][obstacle_mask] = 100
        
        self.grid_received = True
        self.get_logger().info(
            f"Map updated. Origin: ({self.origin_x}, {self.origin_y})"
        )
        
        # Set goal position (top-right corner)
        self.update_goal_position()
        self.try_plan()

    def update_goal_position(self):
        # Set goal to top-right corner in world coordinates
        #self.goal_x = self.origin_x + self.world_size_x
        #self.goal_y = self.origin_y + self.world_size_y
        
        # Convert to grid coordinates to check if position is valid
        goal_grid_x, goal_grid_y = self.world_to_grid(self.goal_x, self.goal_y)
        
        # If goal position is occupied, find nearest free cell
        if goal_grid_x is None or goal_grid_y is None or not self.is_cell_free(goal_grid_x, goal_grid_y):
            self.get_logger().warn("Goal position occupied, finding nearest free cell")
            self.find_nearest_free_goal()

    def find_nearest_free_goal(self):
        # Start from top-right corner and spiral inward until free cell is found
        for radius in range(1, min(self.width, self.height)):
            for dx, dy in [(-r, r) for r in range(radius)]:
                test_x = self.width - 1 + dx
                test_y = dy
                if 0 <= test_x < self.width and 0 <= test_y < self.height:
                    if self.is_cell_free(test_x, test_y):
                        world_x, world_y = self.grid_to_world(test_x, test_y)
                        self.goal_x = world_x
                        self.goal_y = world_y
                        self.get_logger().info(f"Found free goal at: ({world_x}, {world_y})")
                        return
        self.get_logger().error("Could not find a free goal position")

    def position_callback(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.position_received = True
        self.try_plan()

    def handle_replan_request(self, request, response):
        """Handle service requests to trigger RRT replanning."""
        if request.data:
            self.get_logger().info("Replanning triggered via service.")
            success = self.plan_path()
            response.success = success
            response.message = "Replanning successful." if success else "Replanning failed."
        else:
            self.get_logger().info("Replanning request ignored.")
            response.success = False
            response.message = "Replanning not triggered."
        return response

    def world_to_grid(self, x_world, y_world):
        """Convert world coordinates to grid indices"""
        if self.origin_x is None or self.origin_y is None:
            return None, None
            
        grid_x = int((x_world - self.origin_x) / self.resolution)
        grid_y = int((y_world - self.origin_y) / self.resolution)
        
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return grid_x, grid_y
        return None, None

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates (center of cell)"""
        if self.origin_x is None or self.origin_y is None:
            return None, None
            
        world_x = self.origin_x + (grid_x + 0.5) * self.resolution
        world_y = self.origin_y + (grid_y + 0.5) * self.resolution
        return world_x, world_y

    def is_cell_free(self, grid_x, grid_y):
        """Check if a grid cell is free of obstacles"""
        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return False
        return self.grid[grid_y, grid_x] != 100

    def is_state_valid(self, state):
        """Check if an OMPL state is valid"""
        grid_x, grid_y = self.world_to_grid(state[0], state[1])
        if grid_x is None or grid_y is None:
            return False
        return self.is_cell_free(grid_x, grid_y)

    def try_plan(self):
        if self.grid_received and self.position_received and not self.initial_plan_done:
            if self.plan_path():
                self.initial_plan_done = True
    
    def publish_path(self, path):
        """Publish the planned path."""
        try:
            self.get_logger().info('Creating path message...')
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

        # Convert OMPL path states to ROS path message
            for state in path.getStates():
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(state[0])
                pose.pose.position.y = float(state[1])
                pose.pose.position.z = 0.0
            
            # Set orientation (pointing towards next waypoint)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
            
                path_msg.poses.append(pose)

        # Publish the path
            self.get_logger().info(f'Publishing path with {len(path_msg.poses)} poses')
            self.path_publisher.publish(path_msg)
            self.get_logger().info('Path published successfully')
        
        except Exception as e:
            self.get_logger().error(f'Error publishing path: {str(e)}')
            raise

    def plan_path(self):
    # Check goal position
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().error("Goal position not set")
            return False
    
        self.get_logger().info('='*50)
        self.get_logger().info('Starting path planning...')
        self.get_logger().info(f'Start position: ({self.robot_x}, {self.robot_y})')
        self.get_logger().info(f'Goal position: ({self.goal_x}, {self.goal_y})')

    # Setup OMPL
        space = ob.RealVectorStateSpace(2)
        bounds = ob.RealVectorBounds(2)

    # Set bounds based on map size
        bounds.setLow(0, -1)
        bounds.setHigh(0, self.origin_x + self.world_size_x)
        bounds.setLow(1, -1)
        bounds.setHigh(1, self.origin_y + self.world_size_y)
        space.setBounds(bounds)

        self.get_logger().info('Planning bounds set:')
        self.get_logger().info(f'X bounds: [{self.origin_x}, {self.origin_x + self.world_size_x}]')
        self.get_logger().info(f'Y bounds: [{self.origin_y}, {self.origin_y + self.world_size_y}]')

    # Create space information
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        si.setup()
        self.get_logger().info('Space information configured')

    # Set start and goal
        start = ob.State(space)
        start[0] = self.robot_x
        start[1] = self.robot_y

        goal = ob.State(space)
        goal[0] = self.goal_x
        goal[1] = self.goal_y

        self.get_logger().info('Start and goal states created')

    # Setup problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
        self.get_logger().info('Problem definition set')

    # Create and setup planner
        planner = og.RRTConnect(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        self.get_logger().info('RRTConnect planner configured')

    # Attempt to solve
        self.get_logger().info('Attempting to solve path (timeout: 5.0 seconds)...')
        solved = planner.solve(5.0)  # 5 second timeout

        if solved:
            path = pdef.getSolutionPath()
            states = path.getStates()
        
            self.get_logger().info('='*50)
            self.get_logger().info('Path planning successful!')
            self.get_logger().info(f'Found path with {len(states)} states')
        
        # Log path details
            self.get_logger().info('Path waypoints:')
            for i, state in enumerate(states):
                self.get_logger().info(f'  Point {i}: ({state[0]:.4f}, {state[1]:.4f})')
        
            self.get_logger().info('Publishing path...')
            self.publish_path(path)
            self.get_logger().info('Path published successfully')
            self.get_logger().info('='*50)
            return True
        else:
            self.get_logger().error('='*50)
            self.get_logger().error('Path planning failed!')
            self.get_logger().error(f'Start: ({self.robot_x}, {self.robot_y})')
            self.get_logger().error(f'Goal: ({self.goal_x}, {self.goal_y})')
            self.get_logger().error('='*50)
            return False


def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPlanner()

    try:
        rclpy.spin(rrt_planner)
    except KeyboardInterrupt:
        pass
    finally:
        rrt_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()