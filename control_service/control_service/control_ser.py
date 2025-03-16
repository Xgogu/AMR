import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from my_interfaces.srv import MoveToPosition
from turtlesim.srv import TeleportAbsolute  

class TurtleBotServer(Node):

    def __init__(self, name):
        super().__init__(name)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.get_logger().info('Service is on work')
        self.srv = self.create_service(MoveToPosition, 'move_to_position', self.move_to_position_callback)
        self.current_pose = Pose()

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

    def pose_callback(self, msg):
        self.current_pose = msg  

    def move_to_position_callback(self, request, response):
        target_x = request.x
        target_y = request.y
        self.get_logger().info(f'Teleporting to position: ({target_x}, {target_y})')
        
        self.teleport(target_x, target_y, 0.0)  # 瞬移
        
        response.x = self.current_pose.x
        response.y = self.current_pose.y
        return response

    def teleport(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotServer("service_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
