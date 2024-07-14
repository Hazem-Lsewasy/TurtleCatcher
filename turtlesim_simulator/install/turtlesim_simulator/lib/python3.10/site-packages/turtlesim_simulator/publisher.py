import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from math import pow, atan2, sqrt
from turtlesim.srv import Spawn
import random
import time

class TurtleBot(Node):
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        super().__init__('turtlesim_simulator')

        self.pose = Pose()
        self.flag = False

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.client = self.create_client(Kill, '/kill')
        self.spawn()

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Kill service not available')
            return

        self.req = Kill.Request()
        self.req.name = "my_turtle"  # Modify this to match the turtle's name

    def spawn(self):
        theta_spawn = 0.0
        # Create a client for the /spawn service (assuming turtlesim simulator is 'turtlesim_node')
        client = self.create_client(Spawn, '/spawn')

        # Wait for the service to become available
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Spawn service not available')
            rclpy.shutdown()
            return

        # Create a request object for the spawn service
        self.spawn_req = Spawn.Request()
        self.spawn_req.name = "my_turtle"  # Modify this to match the turtle's name

        # Set desired pose (x, y, and orientation) for the new turtle (adjust values as needed)
        self.spawn_req.x = random.uniform(1,10)  # Set x-coordinate
        self.spawn_req.y = random.uniform(1,10)  # Set y-coordinate
        self.spawn_req.theta = theta_spawn  # Set initial orientation (radians)

        # Send the spawn request to the service
        future = client.call_async(self.spawn_req)

        # Wait for the service response
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Error while calling service: %s' % (str(e)))
            rclpy.shutdown()
            return
  
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta=data.theta
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x,data.y,data.theta)
        self.get_logger().info(msg)
        self.move2goal()

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant = 1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant = 5):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = self.spawn_req.x
        goal_pose.y = self.spawn_req.y
        # goal_pose.theta = 0.0

        vel_msg = Twist()

        if (self.euclidean_distance(goal_pose)>=1):
            vel_msg.angular.z=self.angular_vel(goal_pose)
            vel_msg.linear.x=self.linear_vel(goal_pose)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z=0.0
            self.client.call_async(self.req)
            time.sleep(1)
            self.spawn()

        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot()
    rclpy.spin(node)
    rclpy.shutdown    

if __name__ == '__main__':
    while 1:
        {main()}
        