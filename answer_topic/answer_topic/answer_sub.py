#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy                                     
from rclpy.node   import Node                    
from std_msgs.msg import String                  

class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                   
        self.sub = self.create_subscription(\
            String, "teacher", self.listener_callback, 10)        

    def listener_callback(self, msg):
        for num in range(1,10):                        
            self.get_logger().info('同学“%d”: 已回答"%s"' % (num, msg.data))
            
            
            
def main(args=None):                                 
    rclpy.init(args=args)                         
    node = SubscriberNode("answer_sub")    
    rclpy.spin(node)                                
    node.destroy_node()                     
    rclpy.shutdown()                           
