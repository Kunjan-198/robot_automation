#!/usr/bin/env python
import serial,time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def callback(msg):
	print (msg)
	left=msg.linear.x+(msg.angular.z)
	right=msg.linear.x-(msg.angular.z)
	if(left>1.5):
	    left=1.5
    elif(left<-1.5):
        left=-1.5
	if(right>1.5):
	    right=1.5
    elif(right<-1.5):
        right=-1.5
	PWM1 = ((left+1.5)/3)*512
	PWM2 = ((right+1.5)/3)*512
    arduino.write(PWM1.encode())
	time.sleep(1)
	arduino.write(PWM2.encode())
	time.sleep(1)

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