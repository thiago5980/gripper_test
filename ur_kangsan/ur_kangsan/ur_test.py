import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from .urcontrol import URControl
from std_msgs.msg import Int8
import time

gripper_state = 0

def subscribe_topic_message(self, msg):
    Node.get_logger('test').info('Received message: {0}'.format(msg.data))
    gripper_state = msg.data
      
acc = 0.4
vcc = 0.3

def main(args=None):
    rclpy.init(args=args)

    node = Node('ur_node')
    qos_profile = QoSProfile(depth=10)
    gripper_publisher = node.create_publisher(Int8, 'gripper_sub', qos_profile)
    gripper_subscriber = node.create_subscription(
      Int8,
      'gripper_end',
      subscribe_topic_message,
      qos_profile)
    
    UR_ip = "192.168.1.100"
    robot = URControl(UR_IP=UR_ip)
    
    try:
        rclpy.spin(node)
        initial_value = [0.19777557253837585, -1.7656246624388636, 1.624084774647848, -1.428283469086029, -1.5642641226397913, 0.19385164976119995]
        p1_pose = robot.movej(initial_value[0], initial_value[1], initial_value[2], initial_value[3], initial_value[4], initial_value[5], acc=acc, vcc=vcc)
        gripper_msg = Int8()
        gripper_msg.data = 1
        gripper_publisher.publish(gripper_msg)
        
        while rclpy.ok():
            if gripper_state == 1:
                break
            node.get_logger().info('Gripper data published')
            rclpy.spin_once(node)
            time.sleep(0.05)
        print('gripper end')
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()