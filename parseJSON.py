import json
import numpy as np

data = json.load(open("P1.json"))

bounding_polygon = np.array(data["bounding_polygon"])

print(bounding_polygon)
obstacles = []
try:
    i = 0;
    while(1):
        obstacles.append(np.asmatrix((data["obstacle_"+str(i+1)])))
        i+=1
except KeyError:
    pass

print(obstacles[1])
pos_goal = data["pos_goal"]
pos_start = data["pos_start"]
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_dt = data["vehicle_dt"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]
vel_goal = data["vel_goal"]
vel_start = data["vel_start"]