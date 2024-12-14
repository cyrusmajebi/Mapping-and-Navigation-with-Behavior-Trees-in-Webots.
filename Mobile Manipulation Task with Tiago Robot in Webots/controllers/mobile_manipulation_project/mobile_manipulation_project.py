from matplotlib import pyplot as plt
from os.path import exists 
from scipy import signal

import numpy as np
import py_trees
from py_trees.decorators import Repeat
import ast
import sys
import os


from utils import robot, mapping_waypoints, robot_joint_names, blackboard, \
 DoesMapExist, ReadMap, EndSequence
 
 
#imports the necessary classes from the py_trees library
from py_trees.composites import Sequence, Parallel, Selector
from controller import Robot, Supervisor


from navigation import Navigation #imports the Navigation class.
from mapping import Mapping #imports the Mapping class.
from grasping import Grasping #imports the Grasping class.
from configuration import Configuration #imports the Configuration class.


#set the world timestep for the simulation.
#timestep = int(robot.getBasicTimeStep())
timestep = 16


    
encoders={}
for jname in robot_joint_names:
    if jname == 'gripper_left_finger_joint':
        sname = 'gripper_left_sensor_finger_joint'
    elif jname == 'gripper_right_finger_joint':
        sname = 'gripper_right_sensor_finger_joint'
    else:
        sname = jname + "_sensor"
    encoders[jname] = robot.getDevice(sname)
    encoders[jname].enable(timestep)



blackboard.write('robot', robot) #write the robot to the blackboard
blackboard.write('encoders', encoders) #Write dictionary of encoders to blackboard


#creates a key in the blackboard that checks if the robot has completed all taks.
blackboard.write('Completed all tasks', False)



"""behaviour tree instance (py_tree) consisting of a Sequence node which in turn
contains a Selector node and 3 Sequence nodes."""
tree = Sequence("Main", children= [
           Selector("Does map exist?", children=[
             DoesMapExist("Test for map"),
             Sequence("Navigate and Map", children= [
                 Configuration("Navigation configuration", blackboard),
                 Parallel("Move and Map", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), 
                 children=[
                     Mapping("Map the environment", blackboard),
                     Navigation("Move around the table", blackboard, mapping_waypoints)
                     ])
                   
                 ])
             ], memory=True),
            
            #Node to read the generated cspace map.
            ReadMap("Reading Cspace map"),
            
            #Sequence node to pick and place Jar 1.
            Sequence("Pick and Place Jar 1", children= [
                Configuration("Grasp configuration", blackboard),
                
                Navigation("Move to waypoint", blackboard, 
                                        [(1.12337, 0.565488)] ),
                Grasping("Grasp jar", blackboard),
                Configuration("Grasped configuration", blackboard),
                
                Navigation("Move backwards", blackboard, [(-0.0362, 0.1491)] ),
                Navigation("Move to table", blackboard, 
                        [(0.652, -0.0909), (0.67, -0.46), (0.377, -1.26)] ), 
                Configuration("Turn arm right", blackboard), 
                
                Configuration("Lower torso", blackboard),                                  
                Grasping("Place jar", blackboard),
                Configuration("Lift torso", blackboard),
                
            ]),
            
            #Sequence node to pick and place Jar 2.
            Sequence("Pick and Place Jar 2", children= [
                Configuration("Navigation configuration", blackboard),
            
                Navigation("Move to waypoint", blackboard,[(0.819118, -0.3570393)] ),
                Configuration("Grasp configuration", blackboard),
                Navigation("Move to waypoint", blackboard, [(1.4, 0.0240656)] ),
                 
                Grasping("Grasp jar", blackboard),
                Configuration("Grasped configuration", blackboard),
                Navigation("Move backwards", blackboard, [(0.0987788, -1.29907)] ),
               
                Navigation("Move to table", blackboard, 
                        [(0.224, -2.46)] ),
                Configuration("Turn arm right", blackboard), 
                Configuration("Lower torso", blackboard), 
                                 
                Grasping("Place jar", blackboard),
                Configuration("Lift torso", blackboard),
                
            ]),
            
            #Sequence node to pick and place Jar 3.
            Sequence("Pick and Place Jar 3", children= [
                Configuration("Navigation configuration", blackboard),
                
                Navigation("Move around table to waypoints", blackboard, 
                    [(0.39,-2.91), (-0.72,-3.36), (-1.83,-2.66), 
                     (-1.7, -1.61), (-1.7, -0.48),(-1.65, 0.42), 
                     (-0.55, 0.4), (0.17, 0.26)]),
                
                Configuration("Grasp configuration", blackboard),
                Navigation("Move to waypoint", blackboard, [(1.2, -0.124)] ),
                Grasping("Grasp jar", blackboard),
                Configuration("Grasped configuration", blackboard),
                Navigation("Move backwards", blackboard, [(-0.168881, 0.253306)] ),
                
                Navigation("Move to table", blackboard, 
                                    [(0.542827, -0.667317), (0.127867, -1.61177)] ),
                                
                Configuration("Turn arm right", blackboard), 
                Configuration("Lower torso", blackboard),                                  
                Grasping("Place jar", blackboard),
                Configuration("Lift torso", blackboard),
            ]),
            
            
            #Node to terminate the Behavior Tree instanc. 
            EndSequence("End Main Sequence")

           
                       
        ])
 
 
tree.setup_with_descendants() 
 
 
#main controller loop
while robot.step(timestep) != -1:

    
    tree.tick_once()
    pass
