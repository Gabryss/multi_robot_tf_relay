# Inspired by author: Wim Meeussen
# author: Gabriel Garcia

from typing import Optional
from typing import Union
from typing import List

from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TransformBroadcaster:
    """
    :class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
    """
    def __init__(
        self,
        node: Node,
        qos: Optional[Union[QoSProfile, int]] = None,
        namespace=''
    ) -> None:
        """
        .. function:: __init__(node, qos=None)

            Constructor.

            :param node: The ROS2 node.
            :param qos: A QoSProfile or a history depth to apply to the publisher.
        """
        if qos is None:
            qos = QoSProfile(depth=100)
        self.pub_tf = node.create_publisher(TFMessage, f'{namespace}/tf', qos)

    def sendTransform(
        self,
        transform: Union[TransformStamped, List[TransformStamped]]
    ) -> None:
        """
        Send a transform, or a list of transforms, to the Buffer associated with this TransformBroadcaster.

        :param transform: A transform or list of transforms to send.
        """
        if not isinstance(transform, list):
            if hasattr(transform, '__iter__'):
                transform = list(transform)
            else:
                transform = [transform]
        self.pub_tf.publish(TFMessage(transforms=transform))
