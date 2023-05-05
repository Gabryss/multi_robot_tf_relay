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
        
        self.tf_topics = self.get_robots_tf()
        self.get_logger().info(f'tf_topics : {self.tf_topics}')

        self.tf_listener_list = []
        self.tf_buffer_list = []

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)


        for i in range(len(self.tf_topics)):
            robot = re.search(r"/turtle[0-9]*", self.tf_topics[i]).group()
            # self.get_logger().info(f'TESTTTTTT : {robot}')
            self.tf_buffer_list.append(self.create_subscription(TFMessage, self.tf_topics[i] , self.tf_subscriber_callback, 10))

            # self.tf_listener_list.append(TransformListener(self.tf_buffer_list[i], self, namespace=robot))

        self.get_logger().info(f'tf_listener_list : {self.tf_listener_list}')
        self.get_logger().info(f'tf_buffer_list : {self.tf_buffer_list}')

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)
    
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
    

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                
                #TODO: Add prefix on each robot's tf and publish everything on /tf
                

                # global_tf_msg = self.tf_msg
                # global_tf_msg.child_frame_id = f'{self.tf_msg.transforms[0].header.frame_id}/{self.tf_msg.transforms[0].child_frame_id}' 
                # self.pub_tf.publish(global_tf_msg)
                pass


            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()