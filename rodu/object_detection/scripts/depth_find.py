#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String

class DepthImageViewer:

    def __init__(self):
        rospy.init_node('depth_image_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback)
        self.center_pub = rospy.Publisher('/object_center', Point, queue_size=1)
        self.angle_pub = rospy.Publisher('/object_angle', String, queue_size=1)
        self.depth_image = None
        self.threshold = 665  # 임계값 설정

    def image_callback(self, img_msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            self.depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            # depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # cv_image_normalized = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX)

            # # uint8로 변환
            # cv_image_normalized = cv_image_normalized.astype('uint8')

            # # 더 나은 시각화를 위해 컬러 맵 적용
            # cv_image_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)

            # # 이미지 디스플레이
            # cv2.imshow("Depth Image", cv_image_colormap)

            # 임계값보다 낮은 깊이 값을 필터링
            mask = self.depth_image < self.threshold
            filtered_depth = np.where(mask, self.depth_image, 0)

            # 시각화를 위해 필터링된 깊이 이미지를 8비트로 정규화
            normalized_depth = cv2.normalize(filtered_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            # 더 나은 시각화를 위해 컬러맵 적용
            colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

            # 외곽선 찾기
            contours, _ = cv2.findContours(normalized_depth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 컬러 깊이 이미지에 외곽선과 중심점 그리기
            if contours:
                
                # 면적에 따라 가장 큰 외곽선 찾기
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 최소 면적의 회전된 사각형 찾기
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(colored_depth, [box], 0, (0, 255, 0), 2)

                # 사각형의 기울기 추출 및 조정
                angle = rect[2]
                (width, height) = rect[1]
                if width < height:
                    angle = 90 - angle
                else:
                    angle = -angle
                
                angle = angle if angle >= 0 else angle + 180

                # 기울기 값을 정수로 변환하여 출력
                angle = str(int(angle))

                # 가장 큰 외곽선의 중심점 계산
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # 이미지에 중심점 그리기
                    cv2.circle(colored_depth, (cX, cY), 5, (255, 0, 0), -1)

                    # 중심점 좌표를 퍼블리시
                    center_point = Point()
                    center_point.x = cX
                    center_point.y = cY
                    center_point.z = self.depth_image[cY, cX]  # 중심점의 깊이 값
                    self.center_pub.publish(center_point)
                    self.angle_pub.publish(angle)

                    # 중심점 정보 출력
                    # print("Centroid (x, y, depth):", cX, cY, self.depth_image[cY, cX])

            # 결과 디스플레이
            cv2.imshow('Colored Depth Image', colored_depth)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    viewer = DepthImageViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
