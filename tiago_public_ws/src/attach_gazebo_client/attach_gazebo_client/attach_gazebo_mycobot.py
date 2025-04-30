from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
import argparse

class GripperGazeboClient(Node):
    def __init__(self, model_name, grasp_frame, z_distance):
        super().__init__('attach_gazebo')

        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.client = self.create_client(SetEntityState , '/gazebo/set_entity_state')
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.reentrant_group_1)

        self.action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            self.execute_callback,
            callback_group=self.reentrant_group_1
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = SetEntityState.Request()
        self.req.state.name = model_name  # Use the passed model_name
        self._grasp_frame = grasp_frame
        self._drop_frame = model_name
        self._z_distance = z_distance
        self.attach = 0

    def execute_callback(self, goal_handle):
        position = goal_handle.request.command.position
        if position >= 0.3:
            # Close
            self.attach = 2
            goal_handle.succeed()
        else:
            # Open
            self.attach = 1
            goal_handle.succeed()

        result = GripperCommand.Result()
        result.reached_goal = True
        return result

    def timer_callback(self):
        if self.attach == 1:
            self.get_logger().info(f'Releasing.....{self._drop_frame}, {self.req.state.name}')
            self.req.state.reference_frame = self._drop_frame
            self.req.state.pose.position.z  = 0.0
            self.req.state.pose.position.x = 0.0
            self.future = self.client.call_async(self.req)
            self.attach = 0

        elif self.attach == 2:
            self.get_logger().info(f'Grasping.....{self._grasp_frame}. {self.req.state.name}')
            self.req.state.reference_frame = self._grasp_frame
            self.req.state.pose.position.z  = self._z_distance
            self.req.state.pose.position.x = 0.0
            self.future = self.client.call_async(self.req)

        else:
            pass


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Attach object to end-effector')
    parser.add_argument('grasp_frame', type=str, help='Frame of the robot to connect to')
    parser.add_argument('model_name', type=str, help='Object to grasp')
    parser.add_argument('z_distance', type=float, help='Distance from the grasp frame to attach the model')
    args = parser.parse_args()

    client = GripperGazeboClient(model_name=args.model_name, grasp_frame=args.grasp_frame, z_distance=args.z_distance)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client)

    try:
        executor.spin()
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
