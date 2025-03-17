import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from my_interfaces.srv import MoveToPosition  # 确保该接口已正确定义

class TurtleBotController(Node):
    # """ROS2节点实现海龟模拟器的位置控制服务"""
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # 初始化当前位姿存储
        self.current_pose = Pose()
        
        # 创建位姿订阅
        self._init_pose_subscriber()
        
        # 初始化瞬移服务客户端
        self._init_teleport_client()
        
        # 创建自定义移动服务
        self._init_move_service()
        
        self.get_logger().info(f'节点 [{node_name}] 已启动，服务准备就绪')

    def _init_pose_subscriber(self):
        # """初始化位姿订阅器"""
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self._pose_callback,
            10
        )

    def _init_teleport_client(self):
        # """初始化瞬移服务客户端"""
        self.teleport_client = self.create_client(
            TeleportAbsolute, 
            '/turtle1/teleport_absolute'
        )
        # 等待服务可用
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待瞬移服务上线...')

    def _init_move_service(self):
        # """初始化自定义移动服务"""
        self.service = self.create_service(
            MoveToPosition,
            'move_to_position',
            self._handle_move_request
        )

    def _pose_callback(self, msg):
        # """处理位姿更新"""
        self.current_pose = msg

    def _handle_move_request(self, request, response):
        # """处理移动请求"""
        self.get_logger().info(
            f'收到移动请求 → X: {request.x:.2f}, Y: {request.y:.2f}'
        )
        
        # 执行瞬移操作
        self._execute_teleport(request.x, request.y, 0.0)
        
        # 返回当前位置（注意：此处返回的是移动前的位姿）
        response.x = self.current_pose.x
        response.y = self.current_pose.y
        return response

    def _execute_teleport(self, x, y, theta):
        # """执行瞬移操作"""
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)
        self.get_logger().debug(f'瞬移指令已发送 → X: {x:.2f}, Y: {y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController("turtle_position_service")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('节点被用户中断')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()