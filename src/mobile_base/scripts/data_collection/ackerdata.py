#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import SteeringControllerStatus
import pandas as pd
from datetime import datetime

class ControllerStateSubscriber(Node):
    def __init__(self):
        super().__init__('controller_state_subscriber')
        self.subscription = self.create_subscription(
            SteeringControllerStatus,
            '/ackermann_steering_controller/controller_state',
            self.listener_callback,
            30)
        self.subscription  # 防止在方法外被当作未使用变量处理
        self.data_list = []  # 存储数据的列表

    def listener_callback(self, msg):
        
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # 拆分数组中的每个元素到单独的列
        
        if len(msg.traction_wheels_velocity) >= 2:
            traction_wheels_velocity_1 = msg.traction_wheels_velocity[0]
            traction_wheels_velocity_2 = msg.traction_wheels_velocity[1]
        else:
            traction_wheels_velocity_1 = None
            traction_wheels_velocity_2 = None
        
        if len(msg.steer_positions) >= 2:
            steer_position_1 = msg.steer_positions[0]
            steer_position_2 = msg.steer_positions[1]
        else:
            steer_position_1 = None
            steer_position_2 = None
        
        if len(msg.linear_velocity_command) >= 2:
            linear_velocity_command_1 = msg.linear_velocity_command[0]
            linear_velocity_command_2 = msg.linear_velocity_command[1]
        else:
            linear_velocity_command_1 = None
            linear_velocity_command_2 = None
        
        if len(msg.steering_angle_command) >= 2:
            steering_angle_command_1 = msg.steering_angle_command[0]
            steering_angle_command_2 = msg.steering_angle_command[1]
        else:
            steering_angle_command_1 = None
            steering_angle_command_2 = None


        data = {
           'Timestamp': timestamp,
            'Traction_Wheels_Velocity_1': traction_wheels_velocity_1,
            'Traction_Wheels_Velocity_2': traction_wheels_velocity_2,
            'Steer_Position_1': steer_position_1,
            'Steer_Position_2': steer_position_2,
            'Linear_Velocity_Command_1': linear_velocity_command_1,
            'Linear_Velocity_Command_2': linear_velocity_command_2,
            'Steering_Angle_Command_1': steering_angle_command_1,
            'Steering_Angle_Command_2': steering_angle_command_2,
        }
        self.data_list.append(data)
        self.get_logger().info('数据已记录')

    def save_to_excel(self):
        df = pd.DataFrame(self.data_list)  # 创建DataFrame
        df.to_excel('/home/simon/SA/data_collection/ackermann.xlsx', index=False, engine='openpyxl')
        self.get_logger().info('数据已保存到Excel')

def main(args=None):
    rclpy.init(args=args)
    controller_state_subscriber = ControllerStateSubscriber()
    try:
        rclpy.spin(controller_state_subscriber)
    except KeyboardInterrupt:
        controller_state_subscriber.save_to_excel()
    finally:
        controller_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
