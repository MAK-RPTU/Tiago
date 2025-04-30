import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from attach_gazebo_interfaces.srv import AttachCommand

from math import radians, sin, cos


class AttachServer(Node):
    def __init__(self):
        super().__init__('attach_gazebo')
        self.declare_parameter('reference_frame', 'gripper_left_finger_link')
        self.declare_parameter('model_name', 'aruco_cube')
        self.declare_parameter('z_offset', -0.2)

        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        self.attach_timer = None
        self.service = self.create_service(AttachCommand, 'attach_control', self.handle_request)
        self.get_logger().info('Attach server is ready.')

    def handle_request(self, request, response):
        if request.command == 'close':
            self.get_logger().info('Attach request received.')
            self.attach_timer = self.create_timer(0.05, self.attach_object)
            response.success = True
        elif request.command == 'open':
            self.get_logger().info('Detach request received.')
            if self.attach_timer:
                self.attach_timer.cancel()
            response.success = True
        else:
            self.get_logger().error('Invalid command. Use "open" or "close".')
            response.success = False
        return response

    def attach_object(self):
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.reference_frame = self.reference_frame
        req.state.pose.position.x = 0.044
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = self.z_offset

        # 90-degree X rotation
        angle_rad = radians(90)
        req.state.pose.orientation.x = sin(angle_rad / 2)
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = cos(angle_rad / 2)

        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = AttachServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
