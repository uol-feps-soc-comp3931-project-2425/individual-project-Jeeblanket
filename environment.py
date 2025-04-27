import random
from classes import UAV
from classes import HAP
from classes import UserRequest
from optimisation import PSO
from optimisation import GWO

class SimulationEnvironment:
    def __init__(self, a1, a2, b1, b2, gamma, delta, step, no_uavs, lambda_arrival_rate):
        self.uavs = []
        self.haps = []
        self.user_requests = []
        self.pending_requests = []
        self.time = 0
        self.lambda_arrival_rate = lambda_arrival_rate  # Example requests per unit time
        self.a1 = a1
        self.a2 = a2
        self.b1 = b1
        self.b2 = b2
        self.gamma = gamma
        self.delta = delta
        self.step = step
        self.no_uavs = no_uavs

        self.initialize_network()

    def initialize_network(self):
        # Create HAPs
        self.haps.append(HAP(hap_id=0, position=(0, 0, 20000), communication_range=400000))

        # Create UAVs
        for i in range(10):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 9000)
            self.uavs.append(UAV(uav_id=i, position=position, max_vnfs=5, communication_range=150000))
    
    def generate_user_requests(self):
        num_requests = random.poisson(self.lambda_arrival_rate)
        for i in range(num_requests):
            position = (random.uniform(-100000, 100000), random.uniform(-100000, 100000), 0)
            requested_vnfs = random.sample(range(10), random.randint(1, 3))
            self.user_requests.append(UserRequest(request_id=len(self.user_requests), user_position=position, requested_vnfs=requested_vnfs))

    def optimize_network(self):
        # This would call your optimizer (GWO or PSO)
        gwo_optimizer = GWO(self.uavs, self.haps, self.user_requests)
        gwo_optimizer.optimize()

    def distance(self):
        pass

    def request_collection(self):
        rcl = (self.a1 * (distance(user, uav) / bandwidth(user, uav))) + (self.a2 * (distance(uav, hap) / bandwidth(uav, hap)))
        return rcl
    
    def decision_making(self):
        pass

    def placement(self):
        pass

    def preparation(self):
        pass

    def transmission(self):
        pass

    def process_requests(self):
        processed_latencies = []
    
        while self.pending_requests:
            request = self.pending_requests.pop(0)  # FIFO processing
            rcl = self.request_collection(request)
            dml = self.decision_making(request)
            pl = self.placement(request)
            prep = self.preparation(request)
            tx = self.transmission(request)
        
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
        
        return processed_latencies

    def run_simulation(self):
        print(f"--- Time Step {step} ---")
        self.generate_user_requests()
        self.optimize_network()
        step_latencies = self.process_requests()
        self.store_latencies(step_latencies)