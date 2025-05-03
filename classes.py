from math import sqrt

PARAMS = {
    "VNF_set" : [],
    "U": 500,               # number of UAVs in the system
    "C" : 4,                # max VNFs per UAV
    "R": 5,                 # requests per unit time
    "S": 0.025,             # megabits for vnf message
    "B": 192,               # approx number of bits for new position
    "V_max": 30,            # max UAVs active
    "A_max": 20,            # max VNF activations per interval
    "S_max": 120,           # max UAV movement speed
    "R_v": 150000,          # range UAV in meters
    "R_h": 400000,          # range of HAP in metres
    "deltaT": 1,            # timestep 
    "BW_max_user_uav": 50,  # mbps
    "BW_max_uav_hap": 500,  # mbps
    "latency_coeffs": {
        "alpha1": 0.1, "alpha2": 0.2,   #temp - change these
        "beta1": 0.3, "beta2": 0.000005,
        "gamma1": 0.9, "gamma2": 0.2, 
        "delta1": 0.1
    }
}

class UserRequest:
    def __init__(self, request_id, user_position, requested_vnfs, demand=5, ttl):
        self.request_id = request_id
        self.user_position = user_position  # (x, y, z) on ground (assume z=0)
        self.requested_vnfs = requested_vnfs  # list of VNFs
        self.demand = demand
        self.ttl = ttl

class UAV:
    def __init__(self, uav_id, position):
        self.uav_id = uav_id
        self.position = position  # (x, y, z) tuple
        self.last_movement = 0
        self.max_vnfs = PARAMS["C"]
        self.communication_range = PARAMS["R_v"]
        self.active_vnfs = set()
        self.connected_users = []
        self.current_load = 0
        self.max_capacity = PARAMS["BW_max_user_uav"]
        self.is_active = True
        self.max_move = PARAMS["S_max"] * PARAMS["deltaT"]
    
    def activate_vnf(self, vnf_id):
        if len(self.active_vnfs) < self.max_vnfs:
            self.active_vnfs.add(vnf_id)
    
    def deactivate_vnf(self, vnf_id):
        self.active_vnfs.discard(vnf_id)
    
    def move_to(self, new_position):
        self.last_movement = distance(self.position, new_position)
        self.position = new_position
    
    def move(self):
        res = self.last_movement / PARAMS["S_max"]
        return res

    def can_serve_user(self, user_position):
        # Calculate Euclidean distance
        dist = ((self.position[0] - user_position[0])**2 +
                (self.position[1] - user_position[1])**2 +
                (self.position[2] - user_position[2])**2) ** 0.5
        return dist <= self.communication_range


class HAP:
    def __init__(self, hap_id, position):
        self.hap_id = hap_id
        self.position = position  # (x, y, z) tuple
        self.communication_range = PARAMS["R_h"]

def distance(pos1, pos2):
        return sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)

def bandwidth(distance, link_type):
    if link_type == 'user_uav':
        BW_max = PARAMS["BW_max_user_uav"]   # mbps for User <-> UAV
        R_max = PARAMS["R_v"] # meters
    elif link_type == 'uav_hap':
        BW_max = PARAMS["BW_max_uav_hap"]  # mbps for UAV <-> HAP
        R_max = PARAMS["R_h"] # meters
    else:
        raise ValueError(f"Unknown link type: {link_type}")

    # if distance is greater than max range, bandwidth is effectively 0
    if distance > R_max:
        return 1e-6  # or 0, but 1e-6 avoids division by zero issues later

    # apply formula
    bandwidth = BW_max * (1 - (distance / R_max))
    return bandwidth
