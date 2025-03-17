#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy                           
from rclpy.node import Node                     
from std_msgs.msg import String   

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name) 
        self.num=1                                   
        self.pub = self.create_publisher(String, "teacher", 10)   
        self.timer = self.create_timer(1, self.timer_callback)  
        
    def timer_callback(self):                                                                                                                           
        msg = String()
        msg.data = f'问题 "{self.num}"'
        self.pub.publish(msg)
        self.get_logger().info(f'已发布: {msg.data}')
        self.num += 1    
        
def main(args=None):                         
    rclpy.init(args=args)                           
    node = PublisherNode("answer_pub")     
    rclpy.spin(node)                               
    node.destroy_node()                        
    rclpy.shutdown()                                
