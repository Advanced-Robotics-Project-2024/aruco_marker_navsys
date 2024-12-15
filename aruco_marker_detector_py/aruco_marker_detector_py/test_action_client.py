import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from aruco_marker_detector_msgs.action import ApproachMarker

class ActionManagerClient(Node):
    def __init__(self):
        super().__init__("ActionManagerClient")
        self.client = ActionClient(self, ApproachMarker, 'action_manager')

    def send_goal(self):
        msg = ApproachMarker()
        msg.goal_id = 0
        msg.goal_length = 0.5
        
        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)



def main(args=None):
    rclpy.init(args=args)

    client = ActionManagerClient()

    rclpy.spin(client)

#if __name__ == '__main__':
#    main()
