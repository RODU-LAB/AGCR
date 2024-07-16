#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import can
import time
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import Point
from hunter_msgs.msg import HunterStatus
from std_msgs.msg import String
import set_robot


from tf.transformations import quaternion_from_euler
from tf.transformations import *

robot_x = None
robot_y = None
robot_z = None
#----------------- 그리퍼 관련--------------------#

def setup_can_interface(channel, bitrate): # 그리퍼 can 통신 초기 설정
    """
    Set up the CAN interface with the specified channel and bitrate.
    """
    try:
        bus = can.interface.Bus(channel=channel, bustype='seeedstudio', bitrate=bitrate)
        print(f"CAN interface setup on channel {channel} with bitrate {bitrate}.")
        return bus
    except Exception as e:
        print(f"Error setting up CAN interface: {e}")
        return None

def send_can_message(bus, arbitration_id, data): # can 통신 송신
    """
    Send a CAN message on the specified bus.
    """
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    try:
        bus.send(message)
        print(f"Message sent: {message}")
    except Exception as e:
        print(f"Error sending message: {e}")

# CAN 인터페이스 설정
channel = '/dev/ttyUSB0'  # 사용하는 포트
bitrate = 1000000  # 통신속도 1Mbps

bus = setup_can_interface(channel, bitrate)

# 데이터 배열 정의
close = [0x02, 0x01, 0x20, 0x49, 0x20, 0x00, 0xC8]
open = [0x02, 0x00, 0x20, 0x49, 0x20, 0x00, 0xC8]

def gripper_close(): #그리퍼 닫힘
    send_can_message(bus, arbitration_id=0x00000001, data=close)
    time.sleep(3) 

def gripper_open(): #그리퍼 열림
    send_can_message(bus, arbitration_id=0x00000001, data=open)
    time.sleep(3)

#----------------- 그리퍼 관련--------------------#

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object): # moveit 관련 class

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "nova2_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.001)
        move_group.set_max_acceleration_scaling_factor(0.001)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4): # moveit 화면 및 충돌영역 설정 업데이트 함수

            box_name = self.box_name
            scene = self.scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():

                attached_objects = scene.get_attached_objects([box_name])
                is_attached = len(attached_objects.keys()) > 0

                is_known = box_name in scene.get_known_object_names()

                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True
                
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            return False
    
    def go_to_set(self): # 원점 이동 코드 보류!!
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.0001)
        move_group.set_max_acceleration_scaling_factor(0.0001)


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = -0.1175
        pose_goal.position.y = -0.087
        pose_goal.position.z = 0.8484
        
        # Euler 각도를 quaternion으로 변환
        quaternion = quaternion_from_euler(0.270, 0.653, -0.65326)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        move_group.set_pose_target(pose_goal)

        # 경로 계획 및 실행
        plan = move_group.go(wait=True)
        move_group.stop()

    def go_to_home(self): # 초기 원점 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.0001)
        move_group.set_max_acceleration_scaling_factor(0.0001)


        joint_values = [-90, 0, 0, 0, 90, 45]  # 각도를 직접 입력 (degrees 단위)

        # joint_values를 float로 변환하고 라디안으로 변환
        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        # 조인트 값 설정
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        # 현재 조인트 값과 비교
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_check_front(self): # 로봇 정면 원점 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.001)
        move_group.set_max_acceleration_scaling_factor(0.001)

        joint_values = [-90, 0, 0, 0, 90, 45] 

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_check_front1(self): # 로봇 정면 비전 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.01)
        move_group.set_max_acceleration_scaling_factor(0.01)

        joint_values = [-90, 0, -90, 0, 90, 45]

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_check_left(self): # 로봇 왼쪽 원점 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.01)
        move_group.set_max_acceleration_scaling_factor(0.01)

        joint_values = [0, 0, -90, 0, 90, 45]

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_check_left1(self): # 로봇 왼쪽 원점 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.01)
        move_group.set_max_acceleration_scaling_factor(0.01)

        joint_values = [0, 0, -90, 0, 90, 45]

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
       
    def go_to_check_right(self): # 로봇 오른쪽 원점 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.01)
        move_group.set_max_acceleration_scaling_factor(0.01)

        joint_values = [-180, 0, -90, 0, 90, 45]

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_check_right1(self): # 로봇 오른쪽 비전 위치 이동
        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.01)
        move_group.set_max_acceleration_scaling_factor(0.01)

        joint_values = [-180, 0, -90, 0, 90, 45] 

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self,x,y,z,angle): # 쓰레기 위치로 움직이는 함수
        
        rad_angle = 1+(2*angle/90)
        move_group = self.move_group
        move_group.set_planner_id("RRTConnect")
        move_group.set_planning_time(2)

        move_group.set_max_velocity_scaling_factor(0.001)
        move_group.set_max_acceleration_scaling_factor(0.001)

        current_pose = move_group.get_current_pose().pose
        current_z = current_pose.position.z

        current_joint_values = move_group.get_current_joint_values()
        current_joint_1 = current_joint_values[0]
        current_joint_4 = current_joint_values[3] 
        current_joint_5 = current_joint_values[4]
        current_joint_6 = current_joint_values[5] 

        joint_constraint_1 = moveit_msgs.msg.JointConstraint()
        joint_constraint_1.joint_name = "joint1"  # 1번 축의 관절 이름
        joint_constraint_1.position = current_joint_1  # 현재 위치
        joint_constraint_1.tolerance_above = 0.7854  # +10도 (라디안 값)
        joint_constraint_1.tolerance_below = 0.7854  # -10도 (라디안 값)
        joint_constraint_1.weight = 1.0

        joint_constraint_4 = moveit_msgs.msg.JointConstraint()
        joint_constraint_4.joint_name = "joint4"  # 5번 축의 관절 이름
        joint_constraint_4.position = current_joint_4  # 현재 위치
        joint_constraint_4.tolerance_above = 1.5708  # +45도 (라디안 값)
        joint_constraint_4.tolerance_below = 1.5708  # -45도 (라디안 값)
        joint_constraint_4.weight = 1.0


        joint_constraint_5 = moveit_msgs.msg.JointConstraint()
        joint_constraint_5.joint_name = "joint5"  # 5번 축의 관절 이름
        joint_constraint_5.position = current_joint_5  # 현재 위치
        joint_constraint_5.tolerance_above = 0.7854  # +45도 (라디안 값)
        joint_constraint_5.tolerance_below = 0.7854  # -45도 (라디안 값)
        joint_constraint_5.weight = 1.0

        joint_constraint_6 = moveit_msgs.msg.JointConstraint()
        joint_constraint_6.joint_name = "joint6"  # 5번 축의 관절 이름
        joint_constraint_6.position = current_joint_6  # 현재 위치
        joint_constraint_6.tolerance_above = 1.5708  # +45도 (라디안 값)
        joint_constraint_6.tolerance_below = 1.5708  # -45도 (라디안 값)
        joint_constraint_6.weight = 1.0


        constraints = moveit_msgs.msg.Constraints()
        constraints.joint_constraints.append(joint_constraint_1)
        constraints.joint_constraints.append(joint_constraint_4)
        constraints.joint_constraints.append(joint_constraint_5)
        constraints.joint_constraints.append(joint_constraint_6)    
        move_group.set_path_constraints(constraints)


        # PlanningSceneInterface를 통해 박스 추가 (현재 z축 위쪽)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = current_pose.position.x
        box_pose.pose.position.y = current_pose.position.y
        box_pose.pose.position.z = current_z + 0.6  # 박스 중심을 현재 z 위치 위로 설정

        box_name = "upper_box"
        self.scene.add_box(box_name, box_pose, size=(1.0, 1.0, 0.8)) 
   
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z + 0.12


        euler_angles = (-1.5708*2, 0, (1.5708/2)*rad_angle)  # 1.5708 radians = 90 degrees
        quaternion = quaternion_from_euler(*euler_angles)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2] 
        pose_goal.orientation.w = quaternion[3]

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()
        self.scene.remove_world_object(box_name)
        move_group.clear_path_constraints()

        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

    def go_to_trash_goal(self): # 쓰레기통을 버리는 동작

        move_group = self.move_group
        move_group.set_planning_time(2)
        move_group.set_max_velocity_scaling_factor(0.001)
        move_group.set_max_acceleration_scaling_factor(0.001)

        joint_values = [112, -14, -17, -51, 90, 45] 

        try:
            joint_goal = [math.radians(float(value)) for value in joint_values]
        except ValueError:
            print("Error: Please provide valid joint values.")
            return False

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def add_box1(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정1
        box_name = self.box_name
        scene = self.scene

        # Add the main box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.1
        box_pose.pose.position.y = 0.4 
        box_name = "box1"
        box_size_x, box_size_y, box_size_z = 0.3, 0.4, 0.2
        scene.add_box(box_name, box_pose, size=(box_size_x, box_size_y, box_size_z))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def add_box2(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정2

        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.156
        box_pose.pose.position.y = 0.24
        box_name = "box2"
        scene.add_box(box_name, box_pose, size=(0.64, 0.82, 0.31))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def add_box3(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정3
        scene = self.scene

        # Add the main box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"       
        box_pose.pose.position.z = 0.3
        box_pose.pose.position.y = 0.59
        box_pose.pose.position.x = 0
        box_name = "box3"
        scene.add_box(box_name, box_pose, size=(0.3, 0.02, 0.2))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_box4(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정4
        scene = self.scene

        # Add the main box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.3
        box_pose.pose.position.y = 0.21
        box_pose.pose.position.x = 0
        
        box_name = "box4"
        scene.add_box(box_name, box_pose, size=(0.3, 0.02, 0.2))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_box5(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정5
        scene = self.scene

        # Add the main box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.3
        box_pose.pose.position.y = 0.4
        box_pose.pose.position.x = 0.14
        box_name = "box5"
        scene.add_box(box_name, box_pose, size=(0.02, 0.4, 0.2))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def add_box6(self, timeout=4): # 로봇 충돌 방지를 위한 영역 설정6
        scene = self.scene

        # Add the main box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.3
        box_pose.pose.position.y = 0.4
        box_pose.pose.position.x = -0.14
        box_name = "box6"
        scene.add_box(box_name, box_pose, size=(0.02, 0.4, 0.2))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    
class RobotControl: # 로봇 동작 관련 class
    def __init__(self):

        self.point_sub = rospy.Subscriber("/world_point", Point, self.Point_callback, queue_size=1) # world_point 토픽 수신
        self.hunter = rospy.Subscriber('/hunter_status', HunterStatus, self.status_callback, queue_size=1) # hunter_status 토픽 수신
        self.angle = rospy.Subscriber('/object_angle', String, self.angle_callback, queue_size=1) # object_angle 토픽 수신
        
        set_robot.set_robot() # 브레이크 해제

        self.agcr = MoveGroupPythonInterfaceTutorial()
        self.agcr.add_box1()
        self.agcr.add_box2()
        self.agcr.add_box3()
        self.agcr.add_box4()
        self.agcr.add_box5()
        self.agcr.add_box6()
        self.agcr.go_to_home() # 최초 원점 도달
    
    def robot_move(self): # 메인 로봇 시퀀스
        self.robot_front()
        self.robot_trash()
        self.agcr.go_to_home()
   
    def robot_trash(self): # 쓰레기통을 버리는 동작
        self.agcr.go_to_trash_goal()
        gripper_open()

    def robot_front(self): # 정면 확인 및 쓰레기 파지 동작
        self.agcr.go_to_check_front()
        self.agcr.go_to_check_front1()
        self.agcr.go_to_pose_goal(robot_x,robot_y,robot_z,robot_angle)
        gripper_close()
        self.agcr.go_to_check_front1()

    def robot_left(self): # 좌측면 확인 및 쓰레기 파지 동작
        self.agcr.go_to_check_left()
        self.agcr.go_to_check_left1()
        self.agcr.go_to_pose_goal(robot_x,robot_y,robot_z)
        gripper_close()
        self.agcr.go_to_check_left1()

# callback 함수 설정
    def status_callback(self,status_msg): # 리모콘 버튼 시그널을 통해 로봇팔 동작 실행
        status = status_msg.control_mode

        if status == 0 :
            self.robot_move()
        else : 
            return

    def Point_callback(self, msg): # 쓰레기 위치 받아오는 함수
        global robot_x,robot_y,robot_z

        robot_x = msg.x
        robot_y = msg.y
        robot_z = msg.z

    def angle_callback(self, angle_msg): # 쓰레기 각도 받아오는 함수
        global robot_angle

        robot_angle = float(angle_msg.data)

def main():
    rospy.init_node('robot_control_node')
    node = RobotControl()
    rospy.spin()


if __name__ == '__main__':
    main()
