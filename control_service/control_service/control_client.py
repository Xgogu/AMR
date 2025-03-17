import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import MoveToPosition

class TurtlePositionClient(Node):
    # """ROS2客户端实现海龟位置控制服务调用"""
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self._init_service_client()
        self.request = MoveToPosition.Request()
        self.service_future = None

    def _init_service_client(self):
        # """初始化服务客户端"""
        self.client = self.create_client(
            MoveToPosition, 
            'move_to_position'
        )
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.5):
            self.get_logger().warn('服务端未就绪，等待重连...')

    def send_position_request(self, x, y):
        # """发送位置请求并等待响应"""
        self.request.x = x
        self.request.y = y
        
        self.get_logger().debug(
            f'发送移动请求 → X: {x:.2f}, Y: {y:.2f}'
        )
        
        self.service_future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.service_future)
        
        return self._handle_service_response()

    def _handle_service_response(self):
        # """处理服务端响应"""
        if self.service_future.result() is not None:
            return self.service_future.result()
        self.get_logger().error('服务调用失败: 无返回结果')
        raise RuntimeError('服务请求未返回有效结果')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 参数验证
        if len(sys.argv) != 3:
            raise ValueError("需要两个位置参数: X Y")
            
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        
        client = TurtlePositionClient("turtle_position_client")
        response = client.send_position_request(x, y)
        
        client.get_logger().info(
            f'海龟移动前位置 → X: {response.x:.2f}, Y: {response.y:.2f}'
        )
        
    except (ValueError, IndexError) as e:
        print(f"参数错误: {e}\n使用方法: python3 client.py X Y")
    except Exception as e:
        print(f"运行时错误: {str(e)}")
    finally:
        if rclpy.ok():
            client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()