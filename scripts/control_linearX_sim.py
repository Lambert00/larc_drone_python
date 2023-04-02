#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class ControlLinearX():
    def __init__(self):
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        
        # Msgs
        self.cmd_vel_msg = Twist()
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        
    def do_something(self):
        # Takeoff
        while self.takeoff_pub.get_num_connections()<1: pass
        self.takeoff_pub.publish(self.takeoff_msg)
        rospy.sleep(3)
        
        # Linear X 3s
        self.cmd_vel_msg.linear.x = 0.5
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        rospy.sleep(3)
        
        # Stop move
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        rospy.sleep(3)
        
        while self.land_pub.get_num_connections()<1:pass
        self.land_pub.publish(self.land_msg)

def main():
    rospy.init_node('control_LinearX', anonymous=True)
    node = ControlLinearX()
    node.do_something()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass