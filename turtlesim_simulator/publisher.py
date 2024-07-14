import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from math import pow, atan2, sqrt
import spawner

class TurtleBot(Node):

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        super().__init__('turtlesim_simulator')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.client = self.create_client(Kill, '/kill')

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Kill service not available')
            return

        self.req = Kill.Request()
        self.req.name = "my_turtle"  # Modify this to match the turtle's name
        self.client.call_async(self.req)


        timer_period = 0.5
        self.timer=self.create_timer(timer_period,self.move2goal)

        self.pose = Pose()
        self.flag = False

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta=data.theta
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x,data.y,data.theta)
        self.get_logger().info(msg)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = 2.5
        goal_pose.y = 1.5
        goal_pose.theta = 0.0

        angular_tolerance = 0.01

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        # distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        if abs(self.steering_angle(goal_pose)-self.pose.theta)>angular_tolerance:
            vel_msg.linear.x=0.0
            vel_msg.angular.z=self.angular_vel(goal_pose)
        else:
            vel_msg.angular.z=0.0
            if self.euclidean_distance(goal_pose)>=0.1:
                vel_msg.linear.x=self.linear_vel(goal_pose)
            else:
                vel_msg.linear.x = 0.0
                self.flag =True

        if self.flag:
            vel_msg.angular.z = goal_pose.theta-self.pose.theta
            if abs (goal_pose.theta-self.pose.theta) <= angular_tolerance:
                quit()

        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot()
    rclpy.spin(node)
    rclpy.shutdown    

if __name__ == '__main__':
        main()