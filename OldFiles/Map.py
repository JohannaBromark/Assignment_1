import json
import numpy as np
from obstacleCheck import *

class Map():

    def __init__(self, file):
        self.width = 60
        self.height = 60
        
        data = json.load(open(file))

        self.bounding_polygon = np.array(data["bounding_polygon"])

        self.obstacles = []
        for key in data:
            if "obstacle_" in key:
                self.obstacles.append(np.array((data[key])))

        #Hitboxes for obstacles for an easy test if a node is outside an obstacle and inside the bounding_polygon
        self.hitBoxes = makeBoxes(self.obstacles)
        self.boundingHitBox = [findMaxMin(self.bounding_polygon)]

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

    def isOK(self, point):
        # Check if inside the bounding polygon, inside if
        insidePolygon = isBlocked(point, [self.bounding_polygon], self.boundingHitBox)

        # Check if inside an obstacle
        insideObstacle = isBlocked(point, self.obstacles, self.hitBoxes)

        return insidePolygon and not insideObstacle


