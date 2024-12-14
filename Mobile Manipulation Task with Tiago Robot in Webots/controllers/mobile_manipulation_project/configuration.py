from utils import safe_config, navigation_config, grasping_config, grasped_config, \
                  lower_torso_config, lift_torso_config, arm_right_config
import py_trees
import numpy as np
import sys





class Configuration(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Configuration, self).__init__(name)
        
        self.robot=blackboard.read('robot')
        self.encoders=blackboard.read('encoders')
        self.blackboard=blackboard
        self.name = name
        
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        self.motor_left.setPosition(float('Inf'))
        self.motor_right.setPosition(float('Inf'))
        
        self.left_finger = self.robot.getDevice('gripper_left_finger_joint')
        self.right_finger = self.robot.getDevice('gripper_right_finger_joint')
        
        self.left_finger.enableForceFeedback(self.timestep)
        self.right_finger.enableForceFeedback(self.timestep)
        
        self.joint_1 = self.robot.getDevice('arm_1_joint')
        self.joint_2 = self.robot.getDevice('arm_2_joint')
        self.joint_3 = self.robot.getDevice('arm_3_joint')
        self.joint_4 = self.robot.getDevice('arm_4_joint')
        self.joint_5 = self.robot.getDevice('arm_5_joint')
        self.joint_6 = self.robot.getDevice('arm_6_joint')
        self.joint_7 = self.robot.getDevice('arm_7_joint')
        self.lift_joint = self.robot.getDevice('torso_lift_joint')
        
        self.logger.debug(" %s [Configuration::setup()] " % self.name)
        
   
    def initialise(self):
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)
        
        self.device_handles = {}
        self.keys = None
        self.grasp_counter = 0
        

        self.logger.debug("  %s [Configuration::initialise()]" % self.name)
        #print(self.name)
   
   
    def update(self):
        self.logger.debug("  %s [Configuration::update()]" % self.name)
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        
        self.joint_1_pos = self.encoders["arm_1_joint"].getValue() 
        self.torso_lift_pos = self.encoders["torso_lift_joint"].getValue()
        
        self.left_finger_pos = self.encoders["gripper_left_finger_joint"].getValue()
        self.right_finger_pos = self.encoders["gripper_right_finger_joint"].getValue()
        
        self.left_force = self.left_finger.getForceFeedback()
        self.right_force = self.right_finger.getForceFeedback()
        
                
        if self.name == "Safe configuration":
            self.keys = safe_config.keys()
            self.robot_joints = safe_config


        elif self.name == "Navigation configuration":
            self.keys = navigation_config.keys()
            self.robot_joints = navigation_config
      
                
        elif self.name == "Grasp configuration":
            self.keys = grasping_config.keys()
            self.robot_joints = grasping_config

                
        elif self.name == "Grasped configuration":
            self.keys = grasped_config.keys()
            self.robot_joints = grasped_config
          
                
        elif self.name == "Move backwards":
            self.keys = grasped_config.keys()
            self.robot_joints = grasped_config
                
                
        elif self.name == "Move to table":
            self.keys = grasped_config.keys()
            self.robot_joints = grasped_config

                
        elif self.name == "Lower torso":
            self.keys = lower_torso_config.keys()
            self.robot_joints = lower_torso_config
                
                
        elif self.name == "Turn arm right":
            self.keys = arm_right_config.keys()
            self.robot_joints = arm_right_config
                
        
        elif self.name == "Lift torso":
            self.keys = lift_torso_config.keys()
            self.robot_joints = lift_torso_config
      
      
        for key in self.keys:
            m = self.robot.getDevice(key)
            self.device_handles[m] = self.robot_joints[key]
            m.setPosition(self.robot_joints[key])
                               
                               
                
        total_error = sum([ (self.robot_joints[joint_name] - \
                            self.encoders[joint_name].getValue())**2 \
                      for joint_name in self.robot_joints.keys()
                  ])

        if total_error < 0.001:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        pass
        
    
    
    
    