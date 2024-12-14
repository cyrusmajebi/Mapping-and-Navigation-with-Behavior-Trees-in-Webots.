from controller import Robot, Supervisor
from os.path import exists 
import numpy as np
import py_trees
import sys
import os



#robot instance (in supervisor mode)
robot = Supervisor()
display = robot.getDevice('display')


motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')


#Initial set of waypoints
mapping_waypoints = [ (0.61,-0.61), (0.42, -1.82), (0.39,-2.91),  
                      (-0.72,-3.36), (-1.83,-2.66), (-1.7, -1.61), (-1.7, -0.48), 
                      (-1.65, 0.42), (-0.55, 0.4), (0.17, 0.26)]


robot_joint_names = [
    'torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
    'gripper_left_finger_joint', 'gripper_right_finger_joint',
     'head_1_joint', 'head_2_joint']
 
 
 #Create blackboard class
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data.get(key)


blackboard = Blackboard() #blackboard instance

 
#Class to check if a map exists
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        file_exists = exists('cspace.npy')
        #print("Checking for map")
        
        if (file_exists):
            print("Map already exists")
            blackboard.write('map exists', True)
                        
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            blackboard.write('map exists', False)
            return py_trees.common.Status.FAILURE




class ReadMap(py_trees.behaviour.Behaviour):
    def update(self):
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        file_exists = exists('cspace.npy')

        if (file_exists):
            map = np.load('cspace.npy')
            print("Reading map")
            for i in range(map.shape[0]):
                for j in range(map.shape[1]):
                    # Draw map on display
                    if map[i][j] == True:
                        display.setColor(0xFFFFFF)
                        display.drawPixel(i,j)
                        
            return py_trees.common.Status.SUCCESS
        else:
            print("Error reading map")
            return py_trees.common.Status.FAILURE


class EndSequence(py_trees.behaviour.Behaviour):
    def update(self):
        print("Completed all tasks...stopping simulation")
        blackboard.write('Completed all tasks', True)
        motor_left.setVelocity(0.0)
        motor_right.setVelocity(0.0)
        sys.exit(0)
        # return py_trees.common.Status.FAILURE




# A function that maps world coordinates to map coordinates
def world2map(x_w,y_w):
    
   x_w_origin = -2.16
   y_w_origin = 1.78
   p_x_origin = 0.0
   p_y_origin = 0.0
   
   x_w_end = 2.42
   y_w_end = -4.07
   p_x_end = 199.0
   p_y_end = 299.0
   
   px = int(( ((x_w - x_w_origin) * (p_x_end - p_x_origin)) / \
         (x_w_end - x_w_origin)) + p_x_origin)
         
   py = int(( ((y_w - y_w_origin) * (p_y_end - p_y_origin)) / \
         (y_w_end - y_w_origin)) + p_y_origin)
   
   px = min(px, 199)
   py = min(py, 299)
   px = max(px, 0)
   py = max(py, 0)
   
   return [px,py]


# A function that maps map coordinates to world coordinates
def map2world(p_x,p_y):
    
    
   p_x_origin = 0.0
   p_y_origin = 0.0
   x_w_origin = -2.16
   y_w_origin = 1.78
   
   p_x_end = 199.0
   p_y_end = 299.0
   x_w_end = 2.42
   y_w_end = -4.07

   
   x_w = ( ((p_x - p_x_origin) * (x_w_end - x_w_origin)) / \
         (p_x_end - p_x_origin)) + x_w_origin
         
   y_w = ( ((p_y - p_y_origin) * (y_w_end - y_w_origin)) / \
         (p_y_end - p_y_origin)) + y_w_origin
   
   
   return [x_w,y_w]
   
   




safe_config = {
           'torso_lift_joint' : 0.27,
           'arm_1_joint' : 2.68,
           'arm_2_joint' : 1.02,
           'arm_3_joint' : 0,
           'arm_4_joint' : 0,
           'arm_5_joint' : 0,
           'arm_6_joint' : 0,
           'arm_7_joint' : 0,
           'gripper_left_finger_joint' : 0.045,
           'gripper_right_finger_joint': 0.045,
           'head_1_joint':0,
           'head_2_joint':0
 }
 
 


navigation_config = {
           'torso_lift_joint' : 0.27,
           'arm_1_joint' : 1.597,
           'arm_2_joint' : 1.02,
           'arm_3_joint' : 0,
           'arm_4_joint' : -0.32,
           'arm_5_joint' : 0,
           'arm_6_joint' : 0,
           'arm_7_joint' : 1.57,
           'gripper_left_finger_joint' : 0.045,
           'gripper_right_finger_joint': 0.045,
           'head_1_joint':0,
           'head_2_joint':0
}

 
 

grasping_config = {
           'torso_lift_joint' : 0.27,
           'arm_1_joint' : 1.597,
           'arm_2_joint' : 0,
           'arm_3_joint' : 0,
           'arm_4_joint' : 0,
           'arm_5_joint' : 0,
           'arm_6_joint' : 0,
           'arm_7_joint' : 1.57,
           'gripper_left_finger_joint' : 0.045,
           'gripper_right_finger_joint': 0.045,
           'head_1_joint':0,
           'head_2_joint':0
}


grasped_config = {
           'torso_lift_joint' : 0.27,
           'arm_1_joint' : 1.597,
           'arm_2_joint' : 1.02,
           'arm_3_joint' : 0,
           'arm_4_joint' : 0.98,
           'arm_5_joint' : 0,
           'arm_6_joint' : 0,
           'arm_7_joint' : 1.57,
           'head_1_joint':0,
           'head_2_joint':0
}



lower_torso_config = {
            'torso_lift_joint' : 0.09,
            'arm_2_joint' : 0,
            'arm_4_joint' : 0,
                      }  
                      
lift_torso_config = {
            'torso_lift_joint' : 0.27,
            'arm_2_joint' : 1.02,
            'arm_4_joint' : 0.98,
                      }                                                      
arm_right_config = {'arm_1_joint' : 0.1}
