import rclpy, math, transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Twist
from tello_msgs.srv import TelloAction  # Import the service message


class Drone(Node):

    def __init__(self):
        super().__init__('drone')
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

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
        q = transformations.quaternion_from_euler(0, 0, float(1.57079632679))
        self.current_pose.orientation.w = q[0]
        self.current_pose.orientation.x = q[1]
        self.current_pose.orientation.y = q[2]
        self.current_pose.orientation.z = q[3]

        self.send_takeoff_request()
        self.change_altitude(14)

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
        m_time = distance / 4
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.8)

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
