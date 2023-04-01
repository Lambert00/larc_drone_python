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
        
        msg = Empty()
        self.takeoff_pub.publish(msg)
        
        self.goal = 0
        
    def alt_callback(self, value):
        
        if cv2.waitKey(1) & 0xFF == ord('g'): self.goal = 1
        if cv2.waitKey(1) & 0xFF == ord('l'): self.goal = 0.4
        
        current = value.altitude
        self.control_altitude(current, self.goal)
    
    def control_altitude(self, current, goal):
        vel_msg = Twist()
        
        # Control de altitud
        Kp = 0.1
        altError = goal - current
        
        # Velocidad del dron
        vel_msg.linear.z = Kp*altError
        
        # si el goal esta a la altura del land y el valor absoluto del error de velocidad es menor que 0.1 manda land
        if goal == 0.4 and abs(vel_msg.linear.z) < 0.1: 
            land_msg = Empty()
            self.land_pub.publish(land_msg)

            
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