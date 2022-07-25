#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import serial,time

def callback(msg):
	print (msg)
	left=msg.linear.x*(0.07)+(msg.angular.z*(0.07))
	right=msg.linear.x*(0.07)-(msg.angular.z*(0.07))
	pwm1=int()
        arduino.write(cmd.encode())
	time.sleep(3)
    
if __name__ == '__main__':
    
    print('Running. Press CTRL-C to exit.')
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(0.1) #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
                while True:
		    sub =  rospy.Subscriber('cmd_vel', Twist, callback)
            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")

