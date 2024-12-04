# SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import asyncio
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from .submodules.aruco_marker_detector import ArucoMarkerDetector
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
# from rclpy.action import ActionServer
# from std_srvs.srv import Trigger
from aruco_marker_detector_msgs.msg import MarkerInfos
# from aruco_marker_detector_msgs.action import TowardMarker
# from rclpy.executors import MultiThreadedExecutor
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TowardArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('toward_aruco_marker_node')
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_cb,
            2)
        self.drew_marker_image_pub = self.create_publisher(
                Image, 'image/drew_marker', 2)
        self.cmd_vel_pub = self.create_publisher(
                Twist, 'cmd_vel', 2)
        # self.toward_marker_srv = self.create_service(
        #         Trigger, "toward_marker", self.toward_marker_cb)
        
        self.marker_infos_pub = self.create_publisher(
                MarkerInfos, 'marker_infos', 2)

        self.tfb = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.init_image = False
        self.cmd_vel_k = 0.3

        self.centers = []
        self.ids = []

        self.cancel_monitering_time = 5.0
        self.timer_rate = 20.0

        # self.angle_of_view = 55.0
        # self.declare_parameter('angle_of_view', self.angle_of_view)
        # self.get_parameter('angle_of_view', self.angle_of_view)

    def image_cb(self, msg):
        if(self.init_image is False):
            self.aruco_marker_detector = ArucoMarkerDetector(msg.width, msg.height)
            self.init_image = True

        self.get_logger().debug(f'(width, height)=({msg.width, msg.height})')
        raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.centers, self.ids, drew_marker, XYZ, RPY = self.aruco_marker_detector.detect_marker(raw)
        marker_infos = MarkerInfos()
        for m_id, xyz, rpy in zip(self.ids, XYZ, RPY):
            marker_infos.id.append(int(m_id))
            marker_infos.x.append(xyz[2])
            marker_infos.y.append(xyz[0])
            marker_infos.t.append(rpy[1])
        # msg = MarkerInfoArray(marker_infos=marker_infos)
        self.marker_infos_pub.publish(marker_infos)

            
        cvt_ros = self.bridge.cv2_to_imgmsg(drew_marker)
        self.drew_marker_image_pub.publish(cvt_ros)
        #     print(ids)
    
    # ARマーカーを見つけて、マーカーがカメラの
    # 中心に来るまで機体を旋回するサービス
    def rotate_timer_cb(self):
        print(self.ids)
        if(self.ids):
            print("Detected")
            self.rotate_timer_stop()
        print("Not detected")
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = self.cmd_vel_k
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def rotate_timer_start(self):
        self.timer = self.create_timer(1/self.timer_rate, self.rotate_timer_cb)
    
    def rotate_timer_stop(self):
        self.timer.cancel()
        # self.timer = None()

    def toward_marker_cb(self, request, responce):
        cmd_vel_msg = Twist()
        detected_id = -1
        result_srv = False
        on_cancel_monitering_timer = False
        sum_time = 0.0
        pre_time = time.time()
        init_detection = False
        self.rotate_timer_start();
        responce.success = True
        responce.message = "1"
        # ARマーカーが見つかるまで旋回

        ## ARマーカーがカメラの中心になるまで旋回
        #while(True):
        #    if(init_detection is False):
        #        detected_id = self.ids[0]
        #        init_detection = True 
        #        
        #    id_index = self.aruco_marker_detector.rtn_id_index(detected_id)

        #    # ARマーカーが検知し続けられている場合
        #    if(id_index is not None):
        #        print("Detected")
        #        if(on_cancel_monitering_timer is True):
        #            on_cancel_monitering_timer = False
        #            sum_time = 0.0

        #        center_of_id = self.centers[id_index]
        #        position_side = self.aruco_marker_detector.position_side(center_of_id[0])

        #        # 調整の必要がなくなったら成功と判断しループ終了
        #        if(position_side == 0):
        #            result.success = True
        #            result.id = self.detected_id
        #            break

        #        cmd_vel_msg.angular.z = position_side * cmd_vel_k
        #        self.cmd_vel_pub.publish(cmd_vel_msg)
        #        
        #    # ARマーカーを見落とした場合、監視タイマーを開始
        #    else:
        #        print("Lost")
        #        if(on_cancel_monitering_timer is False):
        #            pre_time = time.time()
        #            on_cancel_monitering_timer = True
        #        # 監視時間がしきい値を超えたら失敗と判断し、サービスを終了
        #        else:
        #            current_time = time()
        #            sum_time += (current_time - pre_time)
        #            pre_time = current_time
        #            if(sum_time > self.cancel_monitering_time):
        #                result.success = False
        #                result.id = -1
        #                break
        #    
        #    await asyncio.sleep(loop_rate)
        #cmd_vel_msg.angular.z = 0.0
        #cmd_vel_pub.publish(cmd_vel_msg)
        return responce

    def publish_tf(self, xyz=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.frame_id = "map"

                
def main(args=None):
    rclpy.init(args=args)
    toward_aruco_marker_node = TowardArucoMarkerNode()
    rclpy.spin(toward_aruco_marker_node)

    toward_aruco_marker_node.destory()
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()
