#!/usr/bin/env python3

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image



offest_x = 0

# PointCloud 콜백 함수
def selected_callback(selected_msg):

    global selected_x, selected_y
    
    selected_x = int(selected_msg.x)
    selected_y = int(selected_msg.y)

# def image_callback(data):
#         try:
#             # Convert ROS Image message to OpenCV image
#             bridge = CvBridge()
#             cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#             # Display image
#             cv2.imshow("RGB Image", cv_image)
#             cv2.waitKey(2)
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")

def pointcloud_callback(pointcloud_msg):

    try:
        # 카메라 좌표계에서 world 좌표계로의 변환을 얻음
        (trans, rot) = listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    gen = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True, uvs=[[selected_x, selected_y]])
    
    
    for p in gen:
        # 카메라 좌표계에서의 점
        camera_point = np.array([p[2], -p[0], -p[1], 1.0])

        # 변환 행렬 생성
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[0:3, 3] = trans

        # world 좌표계로 변환
        world_point = np.dot(transform_matrix, camera_point)

        # camera_link 기준의 Marker 생성
        camera_marker = Marker()
        camera_marker.header = Header(frame_id="base_link")
        camera_marker.type = Marker.SPHERE
        camera_marker.action = Marker.ADD
        camera_marker.pose.position = Point(x=world_point[0], y=world_point[1], z=world_point[2])
        camera_marker.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        camera_marker.scale.x = 0.05
        camera_marker.scale.y = 0.05
        camera_marker.scale.z = 0.05
        camera_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 파란색
        camera_marker_pub.publish(camera_marker)

        point_msg = Point(x=world_point[0]+offest_x, y=world_point[1], z=world_point[2])
        point_pub.publish(point_msg)
        
        break


# ROS 노드 초기화
rospy.init_node('realsense_pointcloud_visualizer')

# tf 리스너 생성
listener = tf.TransformListener()

# PointCloud 데이터 구독자
cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, pointcloud_callback)
selected_sub = rospy.Subscriber("/object_center", Point, selected_callback)
# image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

# Marker를 발행할 퍼블리셔
camera_marker_pub = rospy.Publisher("/visualization_marker_camera", Marker, queue_size=1)

point_pub = rospy.Publisher("/world_point", Point, queue_size=1)

# ROS 스핀
rospy.spin()
