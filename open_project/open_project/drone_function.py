import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class Drone(Node):

    def __init__(self):
        super().__init__('drone')
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(Twist, '/tello_drone/cmd_vel', 10)
        self.goal_pose = None
        self.current_pose = None

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'Received goal pose: {msg.pose}')
        self.goal_pose = msg.pose

    def initial_pose_callback(self, msg):
        self.get_logger().info(f'Received initial pose: {msg.pose}')
        self.current_pose = msg.pose

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.goal_pose:
            self.move_towards_goal()

    def move_towards_goal(self):
        if not self.current_pose or not self.goal_pose:
            return

        twist = Twist()

        # Calculate the difference between current position and goal position
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        dz = self.goal_pose.position.z - self.current_pose.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Simple P-controller
        Kp = 0.5
        twist.linear.x = Kp * dx
        twist.linear.y = Kp * dy
        twist.linear.z = Kp * dz

        # Calculate yaw error
        goal_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        yaw_error = goal_yaw - current_yaw

        twist.angular.z = Kp * yaw_error

        self.publisher_.publish(twist)

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        return yaw

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    rclpy.spin(drone)
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
