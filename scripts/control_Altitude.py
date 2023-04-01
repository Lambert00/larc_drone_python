#!/usr/bin/env python

import cv2
import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class ControlAltitude():
    def __init__(self):
        self.velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.alt_callback, queue_size=10)
        
        self.key = None
        self.goal = 0
        
    def take_off(self):
        # Count para darle tiempo a que llege a su altura takeoff
        count = 1000
        msg = Empty()
        
        while count>0:
            self.takeoff_pub.publish(msg)
            count -= 1
        
    def alt_callback(self, value):
        
        if cv2.waitKey(1) & 0xFF == ord('t'): self.take_off()
        if cv2.waitKey(1) & 0xFF == ord('g'): self.goal = 1
        if cv2.waitKey(1) & 0xFF == ord('l'): self.goal = 0.2
        
        current = value.altitude
        self.control_altitude(current, self.goal)
    
    def control_altitude(self, current, goal):
        vel_msg = Twist()
        
        # Control de altitud
        Kp = 0.1
        altError = goal - current
        
        # Velocidad del dron
        vel_msg.linear.z = Kp*altError
        
        land_msg = Empty()
        if goal == 0.2 and abs(vel_msg.linear.z) < 0.1: self.land_pub.publish(land_msg)
        self.velocity_pub.publish(vel_msg)
        print(current)
        print (vel_msg.linear.z)

    
    
def main():
    rospy.init_node('bebop_plan', anonymous=True)
    node = ControlAltitude()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass