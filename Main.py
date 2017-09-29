
# to get the servo motors class & get x and z coordinates of the object 
from Motors import Motor
from SIFT import Image_Tracker
import numpy as np 
import rospy
from geometry_msgs.msg import Point
import cv2
import time




class Main:

    def __init__(self):

        self.motor_dir = 3
        self.timeout = time.time()

    def listener(self):

        rospy.init_node('objective', anonymous=True)
        rospy.Subscriber('objective', Point, self.Tracking)
        M = Motor()
        while True:

            Speed = 25

            if self.motor_dir == 0:
                M.Move_Right(Speed)
                print("right")

            elif self.motor_dir == 1:   
                #Speed = Speed/2
                M.Move_Fwd(Speed)   
                print("forward")



            elif self.motor_dir == 2:
                M.Move_Left(Speed)
                print("left")

        
            elif self.motor_dir == 3:
                M.Full_Stop()
                print("stop")

            
            if time.time() - self.timeout > 4.5:
                self.motor_dir = 3

           





    def Tracking(self,msg):
     
        x = msg.x # center point of the object 
        z = msg.z # distance of the object

        print(x)
        print(z)

        self.timeout = time.time()

        # defining the bins region 
        bins = [0,200,300,640]
        region = np.digitize(x,bins) -1

        # if the object is on the left side of the screen
        if region == 0:
            self.motor_dir = 0 # motor move Right



        # if the object is in the middle of the screen
        elif region == 1:
            self.motor_dir = 1 # motor move Forward
            if z < 0.85:
                self.motor_dir = 1
                time.sleep(5)
                self.motor_dir = 3 # motor stop
                return


        # if the object is on the right side of the screen
        elif region == 2:
            self.motor_dir = 2 # motor move left



        




if __name__ == "__main__":
     
    M = Main()

    # To launch the program you need to enter "Start"
    Program_Launch = raw_input("Enter: ")
    
    if Program_Launch == "start":
        
        
        S = Image_Tracker()
        M.listener() 

    else:
        print "not starting"     

