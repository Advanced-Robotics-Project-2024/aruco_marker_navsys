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
        self.client.send_goal_async(msg)

def main(args=None):
    rclpy.init(args=args)

    client = ActionManagerClient()

    rclpy.spin(client)

#if __name__ == '__main__':
#    main()
