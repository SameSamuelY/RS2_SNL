#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class KinematicsSetter(Node):
    def __init__(self):
        super().__init__('kinematics_setter')
        self.client = self.create_client(SetParameters, '/move_group/set_parameters')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_group/set_parameters...')
        self.get_logger().info('Service available.')

    def set_kinematics(self, yaml_path):
        with open(yaml_path, 'r') as f:
            content = f.read()
        param = Parameter()
        param.name = 'robot_description_kinematics'
        param.value = ParameterValue(string_value=content, type=ParameterType.PARAMETER_STRING)
        req = SetParameters.Request()
        req.parameters = [param]
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Parameter set: {future.result().results[0].successful}')
        else:
            self.get_logger().error('Failed to set parameter')

def main(args=None):
    rclpy.init(args=args)
    setter = KinematicsSetter()
    yaml_path = '~/git/RS2_SNL/ur_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml'
    setter.set_kinematics(yaml_path)
    rclpy.shutdown()

if __name__ == '__main__':
    main()