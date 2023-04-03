#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class ControlAltitude():
    def __init__(self):
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.emergency_pub = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
        rospy.Subscriber('/bebop2/joy', Joy, self.joy_callback, queue_size=1)
        
        # Msgs
        self.cmd_vel_msg = Twist()
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        self.emergency_msg = Empty()
        
    def joy_callback(self, msg:Joy):
        # Maping on images
        axes =  msg.axes
        buttons = msg.buttons
        
        if buttons[2]: self.send_takeoff()
        if buttons[0]: self.send_land()
        if buttons[8]: self.send_emergency()
        
        self.cmd_vel_msg.linear.x = axes[1]
        self.cmd_vel_msg.linear.y = axes[0]
        self.cmd_vel_msg.linear.z = axes[2]
        self.cmd_vel_msg.angular.z = axes[3]
        
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        
    def send_takeoff(self):
        while self.takeoff_pub.get_num_connections()<1: pass
        self.takeoff_pub.publish(self.takeoff_msg)
        
    def send_land(self):
        while self.land_pub.get_num_connections()<1:pass
        self.land_pub.publish(self.land_msg)
    
    def send_emergency(self):
        while self.emergency_pub.get_num_connections()<1:pass
        self.emergency_pub.publish(self.emergency_msg)

def main():
    rospy.init_node('control_Joy_sim')
    node = ControlAltitude()
    rospy.on_shutdown(node.send_land)
    rospy.spin()

if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass