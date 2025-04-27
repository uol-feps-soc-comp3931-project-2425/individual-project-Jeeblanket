import random
import pandas as pd
from collections import deque
from classes import UAV
from classes import HAP
from classes import UserRequest
from optimisation import PSO
from optimisation import GWO

class SimulationEnvironment:
    def __init__(self, a1, a2, b1, b2, gamma, delta, step, no_uavs, lambda_arrival_rate, max_vnfs):
        self.uavs = []  # set of all UAVs
        self.haps = []  # set of all HAPs
        self.user_requests = [] # set of all generated requests
        self.pending_requests = deque() # queue for attending to requests
        self.time = 0
        self.lambda_arrival_rate = lambda_arrival_rate  # requests per unit time
        self.a1 = a1
        self.a2 = a2
        self.b1 = b1
        self.b2 = b2
        self.gamma = gamma
        self.delta = delta
        self.step = step    # timestep between reconfiguration (??)
        self.no_uavs = no_uavs  # number of UAVs in the system
        self.latency_records = []   # record for analysis
        self.max_vnfs = max_vnfs    # max number of vnfs allowed to be active on a UAV

        self.initialize_network()

    def initialize_network(self):
        # create HAPs
        self.haps.append(HAP(hap_id=0, position=(0, 0, 20000), communication_range=400000))

        # create UAVs
        for i in range(self.no_uavs):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 9000)
            self.uavs.append(UAV(uav_id=i, position=position, max_vnfs=self.max_vnfs, communication_range=150000))
    
    def generate_user_requests(self):
        num_requests = random.poisson(self.lambda_arrival_rate)
        for i in range(num_requests):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 0)
            requested_vnfs = random.sample(range(10), random.randint(1, 3))
            new_request = UserRequest(request_id=len(self.user_requests), user_position=position, requested_vnfs=requested_vnfs)

            self.user_requests.append(new_request)
            self.pending_requests.append(new_request)

    def optimise_network(self):
        # calls GWO
        gwo_optimiser = GWO(self.uavs, self.haps, self.user_requests)
        gwo_optimiser.optimise()
    
    def optimise_vnfs(self):
        # calls PSO
        pso_optimiser = PSO(self.uavs, self.haps, self.user_requests)
        pso_optimiser.optimise()

    def distance(self):
        pass

    def bandwidth(self):
        pass

    def assign_user_to_uav(self, request):
        best_uav = None
        best_distance = float('inf')
 
        for uav in self.uavs:
            if uav.can_serve_user(request.user_position):
                # check if UAV can serve the requested VNF
                dist = self.distance(uav.position, request.user_position)
                if dist < best_distance:
                    best_distance = dist
                    best_uav = uav
        if best_uav:
            best_uav.connected_users.append(request)
            return best_uav
        else:
            # No UAV found - queue for retry
            self.pending_requests.append(request)
            return None


    def request_collection(self, request, uav):
        hap = self.haps[0]  # Assume only one HAP for now
        dist_user_uav = self.distance(uav.position, request.user_position)
        dist_uav_hap = self.distance(uav.position, hap.position)
    
        bw_user_uav = self.bandwidth(dist_user_uav, link_type='user_uav')
        bw_uav_hap = self.bandwidth(dist_uav_hap, link_type='uav_hap')
    
        rcl = (self.a1 * (dist_user_uav / bw_user_uav)) + (self.a2 * (dist_uav_hap / bw_uav_hap))
        return rcl
    
    def decision_making(self):
        self.optimise_network()
        self.optimise_vnfs()

    def placement(self):
        pass

    def preparation(self):
        pass

    def transmission(self):
        pass

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
            dml = self.decision_making(request, assigned_uav)
            pl = self.placement(request, assigned_uav)
            prep = self.preparation(request, assigned_uav)
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
            self.process_requests(step)
        