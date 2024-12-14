from utils import world2map, map2world
from matplotlib import pyplot as plt
from scipy import signal
import numpy as np
import py_trees



   
   
class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        
        self.has_run = False
        self.robot=blackboard.read('robot')
        self.blackboard=blackboard
        self.name = name
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        self.display = self.robot.getDevice('display')
        
        
        self.logger.debug(" %s [Mapping::setup()] " % self.name)
   
    def initialise(self):
        self.logger.debug("   %s [Map::initialise()]" % self.name)
        self.map = np.zeros((200, 300))
        self.angles = np.linspace(4.19/2,-4.19/2,667) 
        self.angles = self.angles[80:len(self.angles)-80]

        print("Mapping the environment")

   
    def update(self):
        print(f"{self.parent.name}: => {self.__class__.__name__} => {self.name}")
        
        self.has_run = True
        x_w = self.gps.getValues()[0] 
        y_w = self.gps.getValues()[1] 
        theta=np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        px, py = world2map(x_w,y_w)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px,py)
        
        w_T_r = np.array([[np.cos(theta),-np.sin(theta), x_w],
                          [np.sin(theta),np.cos(theta), y_w],
                          [0,0,1]])            
        ranges = np.array(self.lidar.getRangeImage())
        ranges = ranges[80:len(ranges)-80]
        ranges[ranges == np.inf] = 100
        
        X_i = np.array([ranges*np.cos(self.angles)+0.202, ranges*np.sin(self.angles),
                        np.ones(len(self.angles))])
                            
        D = np.dot(w_T_r, X_i)
        
       
        for d in D.transpose():
            px, py = world2map(d[0], d[1])
            self.map[px, py] += 0.01
            
            if (self.map[px, py] > 1):
                self.map[px, py] = 1
                
            v = int(self.map[px, py]*255)
            color = ( (v*256**2) + (v*256) + (v) )
            
            self.display.setColor(int(color))
            self.display.drawPixel(px, py)
            
        return py_trees.common.Status.RUNNING
        
        
    def terminate(self, new_status):
    
        if (self.has_run):
            kernel = np.ones((26,26)) 
            
            cmap = signal.convolve2d(self.map, kernel, mode='same')
            cspace = cmap>0.9
            
            plt.figure(0)
            plt.imshow(cmap)
            plt.show(block=False)
            plt.pause(2)
            plt.close()
            
            plt.figure(1)
            plt.imshow(cspace)
            plt.show(block=False)
            plt.pause(2)
            plt.close()
            
        
            np.save('cspace',cspace)

            self.blackboard.write('map exists', True)
           
            
            
            
   
   
   
   
   
   