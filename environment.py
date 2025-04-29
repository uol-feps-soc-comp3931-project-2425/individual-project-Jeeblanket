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
        self.lambda_arrival_rate = PARAMS["R"]  # requests per unit time
        self.step = PARAMS["deltaT"]    # timestep between reconfiguration
        self.no_uavs = PARAMS["U"]  # number of UAVs in the system
        self.latency_records = []   # record for analysis

        self.initialize_network()

    def initialize_network(self):
        # create HAPs
        self.haps.append(HAP(hap_id=0, position=(0, 0, 20000)))

        # create UAVs
        for i in range(self.no_uavs):
            position = (random.uniform(-25000, 25000), random.uniform(-50000, 50000), 9000)
            self.uavs.append(UAV(uav_id=i, position=position))
    
    def generate_user_requests(self):
        num_requests = np.random.poisson(self.lambda_arrival_rate)
        print("Number of requests: " + str(num_requests))
        for i in range(num_requests):
            position = (random.uniform(-25000, 25000), random.uniform(-25000, 25000), 0)
            print(position)
            requested_vnfs = random.sample(range(10), random.randint(1, 3))
            new_request = UserRequest(request_id=len(self.user_requests), user_position=position, requested_vnfs=requested_vnfs)

            self.user_requests.append(new_request)
            self.pending_requests.append(new_request)

    def optimise_network(self):
        # calls GWO
        gwo_optimiser = GWO(self.uavs, self.haps, self.user_requests)
        return gwo_optimiser.optimise()
    
    def optimise_vnfs(self):
        # calls PSO
        pso_optimiser = PSO(self.uavs, self.haps, self.user_requests)
        return pso_optimiser.optimise()

    def reassign_users_after_optimization(self):
        print("Reassigning users after optimization...")

        for uav in self.uavs:
            uav.connected_users.clear()
            uav.current_load = 0  # reset load to reassign properly

        reassigned_requests = []

        for request in self.user_requests:
            best_uav = None
            best_distance = float('inf')

            # enforcing constraint 2.15
            for uav in self.uavs:
                if not uav.is_active:
                    continue

                 # enforces assignment range constraint
                if not uav.can_serve_user(request.user_position):
                    continue

                # enforce VNF availability (constraint 2.12)
                vnfs_needed = set(request.requested_vnfs)
                if not vnfs_needed.issubset(uav.active_vnfs):
                    continue

                # meeting constraint 2.14
                if uav.current_load + request.demand > uav.max_capacity:
                    continue

                dist = distance(uav.position, request.user_position)
                if dist < best_distance:
                    best_distance = dist
                    best_uav = uav

            if best_uav:
                best_uav.connected_users.append(request)
                best_uav.current_load += request.demand
                reassigned_requests.append(request.request_id)
            else:
                print(f"Request {request.request_id} could not be reassigned to any UAV after optimisation")

        print(f"Successfully reassigned {len(reassigned_requests)} users.")

    def assign_user_to_uav(self, request):
        # satisfies constraint defined in 2.11
        best_uav = None
        best_distance = float('inf')
 
        for uav in self.uavs:

            dist = distance(uav.position, request.user_position)
            if dist < best_distance:
                best_distance = dist
                best_uav = uav

        if best_uav:
            best_uav.connected_users.append(request)
            best_uav.current_load += request.demand  # increase load
            return best_uav
        else:
            return None


    def request_collection(self, request, uav):
        hap = self.haps[0]  # assume only one HAP for now
        dist_user_uav = distance(uav.position, request.user_position)
        dist_uav_hap = distance(uav.position, hap.position)
        bw_user_uav = bandwidth(dist_user_uav, link_type='user_uav')
        bw_uav_hap = bandwidth(dist_uav_hap, link_type='uav_hap')
    
        rcl = (PARAMS["latency_coeffs"]["alpha1"] * (PARAMS["S"] / bw_user_uav)) + (PARAMS["latency_coeffs"]["alpha2"] * (PARAMS["S"] / bw_uav_hap))
        return rcl
    
    def decision_making(self):
        deployment = self.optimise_network()
        placement = self.optimise_vnfs()
        dml = deployment + placement
        return dml

    def placement(self, uav):
        uav_movement = PARAMS["latency_coeffs"]["gamma1"] * uav.move()
        dist = distance(uav.position, self.haps[0].position)
        bw = bandwidth(dist, 'uav_hap')
        transmission = PARAMS["latency_coeffs"]["gamma2"] * (PARAMS["B"] / bw)
        pl = uav_movement + transmission
        return pl

    def preparation(self, uav):
        dist = distance(uav.position, self.haps[0].position)
        bw = bandwidth(dist, 'uav_hap')
        prep = (PARAMS["latency_coeffs"]["beta1"] * (PARAMS["S"] / bw)) + PARAMS["latency_coeffs"]["beta2"]
        return prep

    def transmission(self, request, uav):
        dist = distance(uav.position,request.user_position)
        bw = bandwidth(dist, 'user_uav')
        tx = PARAMS["latency_coeffs"]["delta1"] * (PARAMS["S"] / bw)
        return tx
        

    def process_requests(self):
        processed_latencies = []
        collected_requests = []

        # assign users to UAVs and gather info
        while self.pending_requests:
            print(str(len(self.pending_requests)) + " pending requests left")
            request = self.pending_requests.popleft()  # FIFO processing

            assigned_uav = self.assign_user_to_uav(request)
            if not assigned_uav:
                print(f"Request {request.request_id} could not be assigned to UAV initially.")
                # mark this user as dropped, add to final results with penalty
                processed_latencies.append({
                    'request_id': request.request_id,
                    'rcl': 0,
                    'dml': 0,
                    'pl': 0,
                    'prep': 0,
                    'tx': 0,
                    'total': 1e9,
                    'total_no_placement': 1e9
                })
                continue

            rcl = self.request_collection(request, assigned_uav)

            collected_requests.append({
                'request': request,
                'assigned_uav': assigned_uav,
                'rcl': rcl
            })

        # after all users are collected, now call decision_making() ONCE
        dml = self.decision_making()
        self.reassign_users_after_optimization()

        # now process placement, preparation, transmission for each user
        for entry in collected_requests:
            request = entry['request']
            assigned_uav = entry['assigned_uav']
            rcl = entry['rcl']

            if assigned_uav not in self.uavs or not assigned_uav.is_active:
                # UAV became inactive after reoptimisation
                print(f"Warning: UAV {assigned_uav.uav_id} is inactive after optimization, dropping request {request.request_id}.")
                processed_latencies.append({
                    'request_id': request.request_id,
                    'rcl': 0,
                    'dml': dml,
                    'pl': 0,
                    'prep': 0,
                    'tx': 0,
                    'total': 1e9,
                    'total_no_placement': 1e9
                })
                continue

            pl = self.placement(assigned_uav)
            prep = self.preparation(assigned_uav)
            tx = self.transmission(request, assigned_uav)

            total_latency = rcl + dml + pl + prep + tx
            total_no_placement = rcl + dml + prep + tx

            processed_latencies.append({
                'request_id': request.request_id,
                'rcl': rcl,
                'dml': dml,
                'pl': pl,
                'prep': prep,
                'tx': tx,
                'total': total_latency,
                'total_no_placement': total_no_placement
            })

        self.latency_records.extend(processed_latencies)
        return processed_latencies

    def run_simulation(self):
        print("--- Simulation Begin ---")
        # need to repeat this for however many time steps will simulate
        self.generate_user_requests()
        self.process_requests()
        