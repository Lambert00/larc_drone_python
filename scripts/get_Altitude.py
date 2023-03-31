#!/usr/bin/env python

#imports files needed to run
import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
        
def alt_callback(value):
    print(value.altitude)

def main():
    rospy.init_node('bebop_plan', anonymous=True)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, alt_callback, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass