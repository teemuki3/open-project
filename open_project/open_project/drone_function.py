import rclpy
import math
import transformations
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped
from tello_msgs.srv import TelloAction
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


class Drone(Node):

    def __init__(self):
        super().__init__('drone')
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Odometry publisher
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # Transform broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # Timer to publish odometry
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create service client
        self.client = self.create_client(TelloAction, '/solo/tello_action')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.goal_pose = None
        self.current_pose = Pose()
        self.current_pose.position.x = float(1)
        self.current_pose.position.y = float(0)
        self.current_pose.position.z = float(1)
        q = transformations.quaternion_from_euler(0, 0, 0)
        self.current_pose.orientation.w = q[0]
        self.current_pose.orientation.x = q[1]
        self.current_pose.orientation.y = q[2]
        self.current_pose.orientation.z = q[3]

        # Drone state
        self.vx = 0.0  # Simulated linear velocity in x
        self.vy = 0.0  # Simulated linear velocity in y
        self.vz = 0.0  # Simulated linear velocity in z
        self.vth = 0.0  # Simulated angular velocity around z-axis

        self.send_takeoff_request()
        self.move_pattern()

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()

        # Create transform from odom to base_link
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = '/odom'
        odom_trans.child_frame_id = f'{self.get_namespace()}/base_link'

        odom_trans.transform.translation.x = self.current_pose.position.x
        odom_trans.transform.translation.y = self.current_pose.position.y
        odom_trans.transform.translation.z = self.current_pose.position.z
        odom_quat = transformations.quaternion_from_euler(0, 0, self.current_pose.orientation.z)
        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        # Publish the transform
        self.odom_broadcaster.sendTransform(odom_trans)

        # Create and publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = '/odom'

        odom.pose.pose = self.current_pose

        odom.child_frame_id = f'{self.get_namespace()}/base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.vth

        self.odom_publisher_.publish(odom)

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'Received goal pose: {msg.pose}')
        self.goal_pose = msg.pose
        self.move_towards_goal()

    def update_current_pose(self):
        self.current_pose.position = self.goal_pose.position
        self.current_pose.orientation = self.goal_pose.orientation

    def move_towards_goal(self):

        # Calculate the difference between current position and goal position
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate yaw difference
        goal_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        d_yaw = math.degrees(goal_yaw - current_yaw)
        if d_yaw < -180:
            d_yaw += 360
        self.get_logger().info(f'Turning: {d_yaw}°, then moving {distance} m')
        self.turn(d_yaw)
        self.move_forward(distance)
        self.update_current_pose()

    def get_yaw_from_pose(self, pose):
        orientation_q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        _, _, yaw = transformations.euler_from_quaternion(orientation_q)
        return yaw

    def turn(self, angle_in_degrees=90):
        request = TelloAction.Request()
        if angle_in_degrees < 0:
            request.cmd = 'rc 0 0 0 0.3'
            m_time = 10 * -angle_in_degrees/360
        else:
            request.cmd = 'rc 0 0 0 -0.3'
            m_time = 10 * angle_in_degrees/360
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(1.2)

    def move_forward(self, distance=1):
        request = TelloAction.Request()
        request.cmd = 'rc 0.4 0 0 0'
        m_time = distance / 2
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.8)

    def move_pattern(self):
        # Move in a simple pattern (e.g., a square or back-and-forth) to cover the maze
        self.change_altitude(14)  # Rise to 2 meters
        for _ in range(4):
            self.move_forward(2)
            self.turn(90)

    def change_altitude(self, change_value):
        if change_value != 0:
            request = TelloAction.Request()
            if change_value < 0:
                request.cmd = 'rc 0 0 -0.4 0'
            else:
                request.cmd = 'rc 0 0 0.4 0'
            m_time = abs(change_value / 4)
            self.client.call_async(request)
            time.sleep(m_time)
            request.cmd = 'rc 0 0 0 0'
            self.client.call_async(request)
            time.sleep(0.8)

    def send_takeoff_request(self):
        # Prepare request message
        request = TelloAction.Request()
        request.cmd = 'takeoff'
        self.client.call_async(request)
        time.sleep(4)


def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    rclpy.spin(drone)
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
