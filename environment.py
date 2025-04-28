import numpy as np
import random
from collections import deque
from classes import UAV
from classes import HAP
from classes import UserRequest
from classes import PARAMS
from classes import bandwidth, distance
from optimisation import PSO
from optimisation import GWO

class SimulationEnvironment:
    def __init__(self):
        self.uavs = []  # set of all UAVs
        self.haps = []  # set of all HAPs
        self.user_requests = [] # set of all generated requests
        self.pending_requests = deque() # queue for attending to requests
        self.time = 0
        self.lambda_arrival_rate = PARAMS["R"]  # requests per unit time
        self.step = PARAMS["deltaT"]    # timestep between reconfiguration
        self.no_uavs = PARAMS["U"]  # number of UAVs in the system
        self.latency_records = []   # record for analysis
        self.max_vnfs = PARAMS["C"]   # max number of vnfs allowed to be active on a UAV

        self.initialize_network()

    def initialize_network(self):
        # create HAPs
        self.haps.append(HAP(hap_id=0, position=(0, 0, 20000)))

        # create UAVs
        for i in range(self.no_uavs):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 9000)
            self.uavs.append(UAV(uav_id=i, position=position))
    
    def generate_user_requests(self):
        num_requests = np.random.poisson(self.lambda_arrival_rate)
        for i in range(num_requests):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 0)
            requested_vnfs = random.sample(range(10), random.randint(1, 3))
            new_request = UserRequest(request_id=len(self.user_requests), user_position=position, requested_vnfs=requested_vnfs)

            self.user_requests.append(new_request)
            self.pending_requests.append(new_request)

    def optimise_network(self):
        # calls GWO
        gwo_optimiser = GWO(self.uavs, self.haps, self.pending_requests)
        return gwo_optimiser.optimise()
    
    def optimise_vnfs(self):
        # calls PSO
        pso_optimiser = PSO(self.uavs, self.haps, self.pending_requests)
        return pso_optimiser.optimise()


    def assign_user_to_uav(self, request):
        # satisfies constraint defined in 2.11
        best_uav = None
        best_distance = float('inf')
 
        for uav in self.uavs:
            if not uav.can_serve_user(request.user_position):
                # enforces assignemtn range constraint
                # check if UAV can serve the requested VNF
                print("uav can not serve bc position")
                continue

            # meeting constraint 2.15
            if not uav.is_active:
                print("uav inactive")
                continue  # skip inactive UAVs

            # # Check if UAV has the VNFs active
            # # meeting constraint 2.12
            # vnfs_needed = set(request.requested_vnfs)
            # if not vnfs_needed.issubset(uav.active_vnfs):
            #     print("vnfs not active")
            #     continue  # Cannot serve this request

            # meeting constraint 2.14
            if uav.current_load + request.demand > uav.max_capacity:
                print("capacity would b exceeded")
                continue  # This UAV cannot take more load

            dist = distance(uav.position, request.user_position)
            if dist < best_distance:
                best_distance = dist
                best_uav = uav

        if best_uav:
            best_uav.connected_users.append(request)
            best_uav.current_load += request.demand  # Increase load
            return best_uav
        else:
            # No UAV found - queue for retry
            # self.pending_requests.append(request)
            return None


    def request_collection(self, request, uav):
        hap = self.haps[0]  # Assume only one HAP for now
        dist_user_uav = distance(uav.position, request.user_position)
        dist_uav_hap = distance(uav.position, hap.position)
        bw_user_uav = bandwidth(dist_user_uav, link_type='user_uav')
        bw_uav_hap = bandwidth(dist_uav_hap, link_type='uav_hap')
    
        rcl = (PARAMS["latency_coeffs"]["alpha1"] * (dist_user_uav / bw_user_uav)) + (PARAMS["latency_coeffs"]["alpha2"] * (dist_uav_hap / bw_uav_hap))
        return rcl
    
    def decision_making(self):
        deployment= self.optimise_network()
        placement = self.optimise_vnfs()
        dml = deployment + placement
        return dml

    def placement(self, uav):
        uav_movement = PARAMS["latency_coeffs"]["gamma1"] * uav.move()
        dist = distance(uav.position, self.haps[0].position)
        bw = bandwidth(dist, 'uav_hap')
        transmission = PARAMS["latency_coeffs"]["gamma2"] * (dist / bw)
        pl = uav_movement + transmission
        return pl

    def preparation(self, uav):
        dist = distance(uav.position, self.haps[0].position)
        bw = bandwidth(dist, 'uav_hap')
        prep = (PARAMS["latency_coeffs"]["beta1"] * (dist / bw)) + PARAMS["latency_coeffs"]["beta2"]
        return prep

    def transmission(self, request, uav):
        dist = distance(uav.position,request.user_position)
        bw = bandwidth(dist, 'user_uav')
        tx = PARAMS["latency_coeffs"]["delta1"] * (dist / bw)
        return tx
        

    def process_requests(self):
        processed_latencies = []
    
        while self.pending_requests:
            request = self.pending_requests.popleft()  # FIFO processing


            assigned_uav = self.assign_user_to_uav(request)
            if not assigned_uav:
                # for debugging only, in reality if skipped need to gather next request
                # need to be careful not to enter an infinite loop if all users cannot be assigned
                print("request {request.request_id} could not be assigned to uav") 
                continue
            rcl = self.request_collection(request, assigned_uav)
            dml = self.decision_making()
            pl = self.placement(assigned_uav)
            prep = self.preparation(assigned_uav)
            tx = self.transmission(request, assigned_uav)
        
            total_latency = rcl + dml + pl + prep + tx
        
            processed_latencies.append({
                'request_id': request.request_id,
                'rcl': rcl,
                'dml': dml,
                'pl': pl,
                'prep': prep,
                'tx': tx,
                'total': total_latency
            })

        self.latency_records.extend(processed_latencies)
        return processed_latencies

    def run_simulation(self):
        for step in range(100):
            print(f"--- Time Step {self.step} ---")
            self.generate_user_requests()
            self.process_requests()
        