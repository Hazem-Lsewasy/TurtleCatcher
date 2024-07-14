import rclpy
from geometry_msgs.msg import Pose
from turtlesim.srv import Spawn

def main(args=None):

  rclpy.init(args=args)
  global x
  global y    
  global theta

  # Create a ROS 2 node
  node = rclpy.create_node('my_spawner_node')

  # Create a client for the /spawn service (assuming turtlesim simulator is 'turtlesim_node')
  client = node.create_client(Spawn, '/spawn')

  # Wait for the service to become available
  if not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().error('Spawn service not available')
    rclpy.shutdown()
    return

  # Create a request object for the spawn service
  req = Spawn.Request()
  x = 2.0
  y = 1.0
  theta = 0.0

  # Set desired pose (x, y, and orientation) for the new turtle (adjust values as needed)
  req.x = x  # Set x-coordinate
  req.y = y  # Set y-coordinate
  req.theta = theta  # Set initial orientation (radians)

  # Send the spawn request to the service
  future = client.call_async(req)

  # Wait for the service response
  try:
    response = future.result()
  except Exception as e:
    node.get_logger().error('Error while calling service: %s' % (str(e)))
    rclpy.shutdown()
    return

  # Destroy the node and shutdown ROS 2 communication
  rclpy.shutdown()

if __name__ == '__main__':
  main()
