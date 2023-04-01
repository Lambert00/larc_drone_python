#!/usr/bin/env python
import cv2
import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class FollowCentroids():
    def __init__(self):
        self.velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.alt_callback, queue_size=10)
        
        msg = Empty()
        self.takeoff_pub.publish(msg)
        
    def alt_callback(self, value):
        self.control_altitude(value.altitude)
    
    def control_altitude(self, current):
        vel_msg = Twist()
        # Altitud deseada 
        Goal = 1.5
        # Current = alt_msg.altitude
        Kp = 0.1

        # Control de altitud
        if current < Goal:
            altError = Goal - current
            # Velocidad del dron
            vel_msg.linear.z = Kp*altError
            self.velocity_pub.publish(vel_msg)
        print(current)
        print (vel_msg.linear.z)

def main():
    rospy.init_node('bebop_plan', anonymous=True)
    node = FollowCentroids()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass