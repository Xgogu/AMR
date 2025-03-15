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
        msg.data = '问题"%d"' % self.num
        self.num=self.num+1                                
        self.pub.publish(msg)                                  
        self.get_logger().info('在线回答: "%s"' % msg.data)     
        
def main(args=None):                         
    rclpy.init(args=args)                           
    node = PublisherNode("answer_pub")     
    rclpy.spin(node)                               
    node.destroy_node()                        
    rclpy.shutdown()                                
