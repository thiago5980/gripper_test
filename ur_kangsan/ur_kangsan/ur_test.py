import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from .urcontrol import URControl
from std_msgs.msg import Int8
import time

gripper_state = 0

def subscribe_topic_message(msg):
    global gripper_state
    gripper_state = msg.data
      
acc = 1.2
vcc = 0.6
height = 0.2717 # 0.27
def main(args=None):
    global gripper_state
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
        # print(robot.get_joint_pose('radian')) 
        ########### gripper open ###########
        gripper_msg = Int8()
        gripper_msg.data = 1
        gripper_publisher.publish(gripper_msg)
        
        while rclpy.ok():
            if gripper_state == 1:
                break
            # node.get_logger().info('Gripper data published')
            rclpy.spin_once(node, timeout_sec=0.05)
        print('gripper open')
        gripper_state = 0
        ########### gripper open ###########

        initial_value = [-0.1694396177874964, -1.731063028375143, 1.7764905134784144, -1.6161166630186976, -1.5638740698443812, 2.1997153759002686]
        p1_pose = robot.movej(initial_value[0], initial_value[1], initial_value[2], initial_value[3], initial_value[4], initial_value[5], acc=acc, vcc=vcc)
        _t = robot.get_tcp_pose()
        
        ########### robot move to goal ###########
        p2_pose = robot.movel(_t[0], _t[1], _t[2]-height, _t[3], _t[4], _t[5], acc=acc, vcc=vcc)
        _t = robot.get_tcp_pose()
        time.sleep(4)
        ########### robot move to goal ###########
        
        ########### grip ###########
        gripper_msg = Int8()
        gripper_msg.data = 2
        gripper_publisher.publish(gripper_msg)
        while rclpy.ok():
            if gripper_state == 1:
                break
            # node.get_logger().info('Gripper data published')
            rclpy.spin_once(node, timeout_sec=0.05)
        print('gripper close')
        gripper_state = 0
        time.sleep(4)
        # time.sleep(2)
        ########### grip ###########
        
        ########### move to another goal ###########
        _t = robot.get_tcp_pose()
        p3_pose = robot.movel(_t[0], _t[1], _t[2]+height, _t[3], _t[4], _t[5], acc=acc, vcc=vcc)
        time.sleep(4)
        # time.sleep(2)
        # _t = robot.get_tcp_pose()
        # p3_pose = robot.movel(_t[0], _t[1], _t[2]-height, _t[3], _t[4], _t[5], acc=acc, vcc=vcc)
        # ########### move to another goal ###########
        
        # ########### gripper open ###########
        # gripper_msg = Int8()
        # gripper_msg.data = 1
        # gripper_publisher.publish(gripper_msg)
        # while rclpy.ok():
        #     if gripper_state == 1:
        #         print("in")
        #         break
        #     # node.get_logger().info('Gripper data published')
        #     rclpy.spin_once(node, timeout_sec=0.05)
        # print('gripper end')
        # gripper_state = 0
        # ########### gripper open ###########
        
        # ########### robot move to initial ###########
        # _t = robot.get_tcp_pose()
        # p3_pose = robot.movel(_t[0], _t[1], _t[2]+height, _t[3], _t[4], _t[5], acc=acc, vcc=vcc)
        # ########### robot move to initial ###########
        
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()