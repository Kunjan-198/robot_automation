#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Twist
# import serial
# import time

# def callback(msg):
# 	print (msg)
# 	left=msg.linear.x*(0.07)+(msg.angular.z*(0.07))
# 	right=msg.linear.x*(0.07)-(msg.angular.z*(0.07))
# 	ser.write(str(left).encode('utf-8'))
# 	ser.write(str(right))
	
# rospy.init_node('topic_subscriber')	
# if __name__ == '__main__':
#     ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
#     ser.reset_input_buffer()
# 	sub =  rospy.Subscriber('cmd_vel', Twist, callback)
#     time.sleep(3)	

import serial,time
if __name__ == '__main__':
    
    print('Running. Press CTRL-C to exit.')
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(0.1) #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
                while True:
                    cmd=input("Enter command : ")
                    arduino.write(cmd.encode())
                    #time.sleep(0.1) #wait for arduino to answer
                    while arduino.inWaiting()==0: pass
                    if  arduino.inWaiting()>0: 
                        answer=arduino.readline()
                        print(answer)
                        arduino.flushInput() #remove data after reading
            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")
