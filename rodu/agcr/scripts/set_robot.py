import rospy
import time
from dobot_bringup.srv import EnableRobot

def set_robot(): 
    try:
        
            rospy.wait_for_service('/dobot_bringup/srv/EnableRobot')
            speed1_robot_proxy = rospy.ServiceProxy('/dobot_bringup/srv/EnableRobot', EnableRobot)

            response1 = speed1_robot_proxy([])

            if response1.res == 0 : 
                print(f"Robot : successfully.\n")
            else:
                rospy.logwarn("Failed to setting speed\n")

    except:
        print("\n\033[91mplease input 'int'\033[0m")


