import numpy as np
import random

PARAMS = {
    "VNF_set" : [],
    "C" : 4,                # max VNFs per UAV
    "V_max": 0,            # max UAVs active
    "A_max": 0,            # max VNF activations per interval
    "R_v": 150000,          # range UAV in meters
    "R_h": 400000,
    "BW_max_user_uav": 100, # mbps
    "BW_max_uav_hap": 1000,
    "latency_coeffs": {
        "alpha1": 0.2, "alpha2": 0.4,   #temp - change these
        "beta1": 0.3, "beta2": 0.1,
        "gamma1": 1.2, "gamma2": 0.6, 
        "delta1": 0.3
    }
}

class UserRequest:
    def __init__(self, request_id, user_position, requested_vnfs):
        self.request_id = request_id
        self.user_position = user_position  # (x, y, z) on ground (assume z=0)
        self.requested_vnfs = requested_vnfs  # List of VNFs

class UAV:
    def __init__(self, uav_id, position, max_vnfs, communication_range):
        self.uav_id = uav_id
        self.position = position  # (x, y, z) tuple
        self.max_vnfs = max_vnfs
        self.communication_range = communication_range
        self.active_vnfs = set()
        self.connected_users = []
    
    def activate_vnf(self, vnf_id):
        if len(self.active_vnfs) < self.max_vnfs:
            self.active_vnfs.add(vnf_id)
    
    def deactivate_vnf(self, vnf_id):
        self.active_vnfs.discard(vnf_id)
    
    def move_to(self, new_position):
        self.position = new_position

    def can_serve_user(self, user_position):
        # Calculate Euclidean distance
        dist = ((self.position[0] - user_position[0])**2 +
                (self.position[1] - user_position[1])**2 +
                (self.position[2] - user_position[2])**2) ** 0.5
        return dist <= self.communication_range


class HAP:
    def __init__(self, hap_id, position, communication_range):
        self.hap_id = hap_id
        self.position = position  # (x, y, z) tuple
        self.communication_range = communication_range

class VNF:
    pass

