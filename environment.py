import random
from classes import UAV
from classes import HAP
from classes import UserRequest
from optimisation import PSO
from optimisation import GWO

class SimulationEnvironment:
    def __init__(self):
        self.uavs = []
        self.haps = []
        self.user_requests = []
        self.time = 0
        self.lambda_arrival_rate = 5  # Example requests per unit time

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

    def run_simulation(self):
        for step in range(100):
            print(f"--- Time Step {step} ---")
            self.generate_user_requests()
            self.optimize_network()
            # Clear old requests if needed
            self.user_requests = []