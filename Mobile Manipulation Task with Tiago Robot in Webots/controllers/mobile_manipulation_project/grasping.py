import py_trees
import numpy as np
import sys




class Grasping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Grasping, self).__init__(name)
        
        self.robot=blackboard.read('robot')
        self.encoders=blackboard.read('encoders')
        self.blackboard=blackboard
        self.name = name
        
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.left_finger = self.robot.getDevice('gripper_left_finger_joint')
        self.left_finger.enableForceFeedback(self.timestep)
        
        self.right_finger = self.robot.getDevice('gripper_right_finger_joint')
        self.right_finger.enableForceFeedback(self.timestep)
        
        self.joint_1 = self.robot.getDevice('arm_1_joint')
        self.joint_2 = self.robot.getDevice('arm_2_joint')
        self.joint_4 = self.robot.getDevice('arm_4_joint')
        self.lift_joint = self.robot.getDevice('torso_lift_joint')
                
        
        self.logger.debug(" %s [Navigation::setup()] " % self.name)
        
   
    def initialise(self):
    
        self.device_handles = {}
        self.keys = None
        self.grasp_counter = 0
        self.release_counter = 0

        self.logger.debug("  %s [Navigation::initialise()]" % self.name)
        print(self.name)
   
   
    def update(self):
        self.logger.debug("  %s [Navigation::update()]" % self.name)
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        
        self.joint_1_pos = self.encoders["arm_1_joint"].getValue() 
        self.torso_lift_pos = self.encoders["torso_lift_joint"].getValue()
        
        self.left_finger_pos = self.encoders["gripper_left_finger_joint"].getValue()
        self.right_finger_pos = self.encoders["gripper_right_finger_joint"].getValue()
        
        self.left_force = self.left_finger.getForceFeedback()
        self.right_force = self.right_finger.getForceFeedback()
      
        #Grasp operation
        if self.name == "Grasp jar":

            #Close the left gripper by 0.0045
            if (round((abs(self.left_finger_pos)), 4) > 0.0000):
                self.left_finger.setPosition(self.left_finger_pos-0.0045)
                   
            #Close the right gripper by 0.0045        
            if (round((abs(self.right_finger_pos)), 4) > 0.0000):
                self.right_finger.setPosition(self.right_finger_pos-0.0045)
                
            #Check if the magnitude of the left and right forces are up to 10
            grasp_complete = (abs(self.left_force) >= 10) and (abs(self.right_force) >= 10)
        
        
            if grasp_complete:
                self.grasp_counter += 1
                #print(f"grasp counter => {self.grasp_counter}")
                
                if self.grasp_counter == 10:
                
                    print("Grasped jar successfully!")
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING
                
            else:
                return py_trees.common.Status.RUNNING
              
              
        #Placing/Release operation        
        elif self.name == "Place jar":

            
            self.left_finger.setPosition(0.045)
            self.right_finger.setPosition(0.045)

            release_complete = (self.left_finger_pos > 0.02) and (self.right_finger_pos > 0.02)

            
            if release_complete:
                self.release_counter += 1
                
                if self.release_counter == 20:
                              
                    print("Placed jar successfully!")                    
                    return py_trees.common.Status.SUCCESS
                    
                else:
                    return py_trees.common.Status.RUNNING
                
                
            else:
                return py_trees.common.Status.RUNNING
                 
              
    def terminate(self, new_status):
        pass
        
    
    
    