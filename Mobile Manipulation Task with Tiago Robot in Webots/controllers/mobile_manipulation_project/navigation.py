from utils import world2map
import py_trees
import numpy as np
import sys





class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, waypoints):
        super(Navigation, self).__init__(name)
    
        self.robot=blackboard.read('robot')
        self.blackboard=blackboard
        self.name = name
        self.WP = waypoints
        
        
    def setup(self):
       
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.display = self.robot.getDevice('display')
        
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        self.motor_left.setPosition(float('Inf'))
        self.motor_right.setPosition(float('Inf'))
        
        self.marker = self.robot.getFromDef("marker").getField("translation")
        
        
        self.parent_name = self.parent.name
        self.odometry_color = None
        if self.parent_name == 'Pick and Place Jar 1':
            self.odometry_color = 0xDF00FE
        elif self.parent_name == 'Pick and Place Jar 2':
            self.odometry_color = 0x3EFF15
        elif self.parent_name == 'Pick and Place Jar 3':
            self.odometry_color = 0xFFF857
        
     
        self.logger.debug(" %s [Navigation::setup()] " % self.name)
   
    def initialise(self):
       
        self.index = 0     
        self.device_handles = {}       
        self.keys = None
        
        self.theta_initial = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
         
        self.logger.debug("  %s [Navigation::initialise()]" % self.name)
        #print(self.name)
   
   
    def update(self):
        self.logger.debug("  %s [Navigation::update()]" % self.name)
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        
        x_w = self.gps.getValues()[0] 
        y_w = self.gps.getValues()[1] 
        theta=np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        
        self.marker.setSFVec3f([*self.WP[self.index], 0.02])
        rho = np.sqrt((x_w-self.WP[self.index][0])**2 + (y_w-self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1]-y_w, self.WP[self.index][0]-x_w)-theta
        
        
        self.marker.setSFVec3f([*self.WP[self.index], 0.01])
        
        if (alpha > np.pi):
            alpha = alpha - 2*np.pi  
        elif(alpha < -np.pi):
            alpha = alpha + 2*np.pi
     
     
        if self.name == "Move around the table":
          
            p1 = 5
            p2 = 4
         
            vL = -p1*alpha + p2*rho
            vR = p1*alpha + p2*rho
            
            vL *= 0.5
            vR *= 0.5
            
            vL = max(min(vL, 6.28), -6.28)
            vR = max(min(vR, 6.28), -6.28) 
            
            self.motor_left.setVelocity(vL)
            self.motor_right.setVelocity(vR)
            
            
            if (rho<0.4):
                print("Reached ", self.index, len(self.WP))
                self.index = self.index + 1
                
                if self.index == len(self.WP):
                
                    print("Reached last waypoint")                
                    return py_trees.common.Status.SUCCESS
                    
                else:
                    return py_trees.common.Status.RUNNING
                    
                
                    
            else:
                return py_trees.common.Status.RUNNING
            
            
        #Pick and place navigation
        else:
            
           
            if self.name == "Move backwards":


                vL = -rho
                vR = -rho

                               
            elif self.name == "Move to waypoint":
                p1 = 3.5
                p2 = 2
             
                vL = (-p1*alpha) + (p2*rho)
                vR = (p1*alpha) + (p2*rho)
                
                vL *= 0.25
                vR *= 0.25
                
            elif self.name == "Move to table":
                p1 = 3.5
                p2 = 2
             
                vL = (-p1*alpha) + (p2*rho)
                vR = (p1*alpha) + (p2*rho)
                
                vL *= 0.25
                vR *= 0.25
                
            elif self.name == "Move around table to waypoints":
                p1 = 5
                p2 = 4
             
                vL = (-p1*alpha) + (p2*rho)
                vR = (p1*alpha) + (p2*rho)
                
                vL *= 0.6
                vR *= 0.6
                
    
    
            px, py = world2map(x_w,y_w)
            self.display.setColor(self.odometry_color)
            self.display.drawPixel(px,py)
            
            vL = max(min(vL, 6.28), -6.28)
            vR = max(min(vR, 6.28), -6.28) 
            
                
            self.motor_left.setVelocity(vL)
            self.motor_right.setVelocity(vR)

            if (rho<0.35):
                if (self.index == len(self.WP)-1):

                    return py_trees.common.Status.SUCCESS
                    
                else:
                    self.index += 1
                 
                    return py_trees.common.Status.RUNNING
                    
            else:
                return py_trees.common.Status.RUNNING
                
                

          
    def terminate(self, new_status):
        pass
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    