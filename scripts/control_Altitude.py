#!/usr/bin/env python

import cv2
import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

# rostopic pub --once equivalent on node
# https://get-help.robotigniteacademy.com/t/how-to-publish-once-only-one-message-into-a-topic-and-get-it-to-work/346

# modes = 'altitude', 'landing', ...

class ControlAltitude():
    def __init__(self):
        
        # Publishers and Subscribers
        self.cmd_vel_pub_ = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub_ = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub_ = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.alt_callback, queue_size=10)
        
        # Msgs
        self.cmd_vel_msg = Twist()
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        
        # Variables
        self.minimal_altitude = 0.4
        self.goal = self.minimal_altitude
        self.mode = 'altitude'
        
        # Takeoff action
        while self.takeoff_pub_.get_num_connections()<1: 
            print('sending takeoff msg')
            
        self.takeoff_pub.publish(self.takeoff_msg)
        print('takeoff msg recived')
        
    def alt_callback(self, value:Ardrone3PilotingStateAltitudeChanged):
        
        # NumLock affects the unicode of the keyboard, so the "& 0xFF" avoids inconveniences
        # ord('letter') returns the unicode of the keyboard
        key = cv2.waitkey(1) & 0xFF
        
        if key == ord('g') or key == ord('G'): 
            self.goal = 1.0
            
        if key == ord('l') or key == ord('L'): 
            self.goal = self.minimal_altitude
            self.mode = 'landing'
        
        current = value.altitude
        self.control_altitude(current, self.goal)
    
    def control_altitude(self, current, goal):
        vel_msg = Twist()
        
        # Altitude control
        Kp = 0.1
        altError = goal - current
        vel_msg.linear.z = Kp*altError
        self.velocity_pub.publish(vel_msg)
        
        # landing action
        if self.mode == 'landing' and abs(altError) < 0.1: 
            while self.land_pub_.get_num_connections()<1: 
                print('sending land msg')
                
            self.land_pub_.publish(self.land_msg)
            print('land msg recived')

        
        print(goal, current, altError, vel_msg.linear.z, self.mode)

def main():
    rospy.init_node('bebop_control_altitude', anonymous=True)
    node = ControlAltitude()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass