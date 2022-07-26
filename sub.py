#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import serial,time

def callback(msg):
	print (msg)
	left=msg.linear.x+(msg.angular.z)
	right=msg.linear.x-(msg.angular.z)
	if(abs(left)>1.5):
	    left=(left/abs(left))*1.5
	if(abs(right)>1.5):
	    right=(right/abs(right))*1.5
	PWM1 = (left/1.5)*255
	PWM2 = (right/1.5)*255
        arduino.write(PWM1.encode())
	time.sleep(1)
	arduino.write(PWM2.encode())
	time.sleep(1)

rospy.init_node('Publisher')	
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

