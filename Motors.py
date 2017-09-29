

##############IMPORTS#################
import platform
import os, yaml
import time
import pypot.dynamixel
import sys

######################################

# This Class Controls the motors
class Motor:
    def __init__(self):
        settings = yaml.safe_load(open("settings.yml"))

        # Base Variables
        self.wheels_one = settings['dynamixel']['wheels'][0]
        self.wheels_two = settings['dynamixel']['wheels'][1]
        self.wheels_three = settings['dynamixel']['wheels'][2]
        self.base_tower = settings['dynamixel']['base']
        
        #Dynamixel Setup
        ServoIDs = (self.base_tower,self.wheels_one,self.wheels_two,self.wheels_three)
        
        #Connect to Dynamixels

        base_port = None
        while base_port is None:
            try:
                base_port = pypot.dynamixel.find_port(ServoIDs)
            except IndexError, e:
                print "Failed to connect to base dynamixel, retrying..."
       
        
        # Servo Config
        self.base_io = pypot.dynamixel.DxlIO(base_port,timeout=2)
        self.base_io.enable_torque(ServoIDs)
        self.base_io.set_wheel_mode(ServoIDs)
    
        # Clock wise and Anticlock wise motor speed 
        self.CW_Half = 125
        self.CCW_Half = -125

        self.Stop = 0

    # Set each wheel
    def set_base(self, W1, W2 ,W3):

        self.base_io.set_moving_speed({self.wheels_one:W1})
        self.base_io.set_moving_speed({self.wheels_two:W2})
        self.base_io.set_moving_speed({self.wheels_three:W3})

    # Function to make the base move forward
    def Move_Fwd(self, Speed):

        self.set_base(-Speed,Speed,0)


    # Function to make the base move backward
    def Move_Bwd(self, Speed):

        self.Move_Fwd(-Speed)


    # Function to make the base move Right
    def Move_Right(self, Speed):

        self.Move_Left(-Speed)


    # Function to make the base move Left
    def Move_Left(self, Speed):

        self.set_base(Speed,Speed,-Speed)
        


    # Function to make the arm move up or down
    def Mast_Move(self, Direction):

        if Direction  == -1:
            W4 = self.CW_Half

        elif Direction == 1:
            W4 = self.CCW_Half

        elif Direction  == 0:
            W4 = self.Stop
        
        self.base_io.set_moving_speed({self.base_tower:W4})

    # Function to stop all the motors 
    def Full_Stop(self):

        self.base_io.set_moving_speed({self.base_tower:self.Stop})
        self.set_base(0, 0 ,0)

def main(args):

    Speed = 125


    motor_control = Motor()  

    try:
        while True:
            print('Testing motors...')
            motor_control.Move_Left(Speed)
            time.sleep(5)
            motor_control.Full_Stop()
            break
    except KeyboardInterrupt:
        print("Shutting down")
                  
     
    


if __name__ == "__main__":
    main(sys.argv)
  
