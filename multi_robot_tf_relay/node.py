import math
import re 


import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from multi_robot_tf_relay.transform_listener import TransformListener
from multi_robot_tf_relay.transform_broadcast import TransformBroadcaster

from turtlesim.srv import Spawn
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage



class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value
        
        self.get_logger().info(f'{self.target_frame}')

        self.tf_topics = self.get_robots_tf()
        self.get_logger().info(f'tf_topics : {self.tf_topics}')

        self.tf_listener_list = []
        self.tf_buffer_list = []

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)


        for i in range(len(self.tf_topics)):
            robot = re.search(r"/turtle[0-9]*", self.tf_topics[i]).group()
            self.tf_buffer_list.append(self.create_subscription(TFMessage, self.tf_topics[i] , self.tf_subscriber_callback, 10))

            # self.tf_listener_list.append(TransformListener(self.tf_buffer_list[i], self, namespace=robot))

        self.get_logger().info(f'tf_listener_list : {self.tf_listener_list}')
        self.get_logger().info(f'tf_buffer_list : {self.tf_buffer_list}')

    
    def get_robots_tf(self):
        """
        Get the robot transform topic
        CAUTION: Now it is only based on the turtlesim architecture
        """
        topics = [t[0] for t in self.get_topic_names_and_types()]
        tf_topics = []

        for i in range(len(topics)):
            reg = re.search(r"/turtle[0-9]*/tf", topics[i])

            if reg is not None:
                tf_topics.append(topics[i])
            else:
                continue

        return tf_topics
    

    def tf_subscriber_callback(self, msg):
        self.tf_msgs = msg
        tf_msg = TFMessage()
        global_tf_msg = self.tf_msgs.transforms[0]
        self.get_logger().info(f'{global_tf_msg}')


        global_tf_msg.child_frame_id = f'{self.tf_msgs.transforms[0].child_frame_id}/{self.tf_msgs.transforms[0].header.frame_id}' 
        
        
        tf_msg.transforms = [global_tf_msg]
        
        self.pub_tf.publish(tf_msg)
    


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()