import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import heapq

class TurtlebotNavigator(Node):

    def __init__(self):
        super().__init__('turtlebot_navigator')
        
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None

    def map_callback(self, map_msg):
        self.get_logger().info('Received map')
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.resolution = map_msg.info.resolution
        self.width = map_msg.info.width
        self.height = map_msg.info.height
        self.origin = map_msg.info.origin

        # For simplicity, assume the goal is fixed for this example
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 10.0  # Set further away goal
        goal_pose.pose.position.y = 10.0  # Set further away goal
        goal_pose.pose.position.z = 0.0

        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(f'Published goal pose: {goal_pose.pose}')

        # Perform path planning
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.0

        path = self.plan_path(start_pose.pose.position, goal_pose.pose.position)
        if path:
            self.publish_path(path)
        else:
            self.get_logger().error('No path found')

    def plan_path(self, start, goal):
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        start_grid = (int((start.x - self.origin.position.x) / self.resolution), int((start.y - self.origin.position.y) / self.resolution))
        goal_grid = (int((goal.x - self.origin.position.x) / self.resolution), int((goal.y - self.origin.position.y) / self.resolution))

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_grid)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def get_neighbors(self, pos):
        neighbors = [
            (pos[0] + 1, pos[1]),
            (pos[0] - 1, pos[1]),
            (pos[0], pos[1] + 1),
            (pos[0], pos[1] - 1),
        ]
        valid_neighbors = []
        for n in neighbors:
            if 0 <= n[0] < self.width and 0 <= n[1] < self.height and self.map_data[n[1]][n[0]] == 0:
                valid_neighbors.append(n)
        return valid_neighbors

    def publish_path(self, grid_path):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for grid in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = grid[0] * self.resolution + self.origin.position.x
            pose.pose.position.y = grid[1] * self.resolution + self.origin.position.y
            path.poses.append(pose)
        
        self.path_publisher.publish(path)
        self.get_logger().info(f'Published planned path with {len(path.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    turtlebot_navigator = TurtlebotNavigator()
    rclpy.spin(turtlebot_navigator)
    turtlebot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
