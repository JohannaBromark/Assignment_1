import json
import numpy as np
import matplotlib as plt

class Map():

    def __init__(self, file):
        self.width = 10
        self.height = 10
        
        data = json.load(open(file))

        self.bounding_polygon = np.array(data["bounding_polygon"])

        #print(self.bounding_polygon)
        self.obstacles = []
        try:
            i = 0;
            while(1):
                self.obstacles.append(np.asmatrix((data["obstacle_"+str(i+1)])))
                i+=1
        except KeyError:
            pass

        #print(self.obstacles[1])
        self.pos_goal = data["pos_goal"]
        self.pos_start = data["pos_start"]
        self.vehicle_L = data["vehicle_L"]
        self.vehicle_a_max = data["vehicle_a_max"]
        self.vehicle_dt = data["vehicle_dt"]
        self.vehicle_omega_max = data["vehicle_omega_max"]
        self.vehicle_phi_max = data["vehicle_phi_max"]
        self.vehicle_t = data["vehicle_t"]
        self.vehicle_v_max = data["vehicle_v_max"]
        self.vel_goal = data["vel_goal"]
        self.vel_start = data["vel_start"]

##        self.area = plt.path(self.bounding_polygon)
##        self.obstacle_paths = []
##        for obstacle in self.obstacles:
##            self.obstacle_paths.append(plt.path(obstacle))

    def isBlocked(self, point):
        return False
