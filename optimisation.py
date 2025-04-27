import math
import random

class GWO:
    def __init__(self, uavs, haps, request):
        self.uavs = uavs
        self.haps = haps
        self.request = request

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)

    
    def fitness(self, uav_positions):
        # Calculate total latency based on UAV positions
        # (use a simplified or estimated latency here)
        total_latency = 0
        for request in self.requests:
            best_latency = float('inf')
            for pos in uav_positions:
                dist = self.calculate_distance(pos, request.user_position)
                latency = dist  # For now assume latency ~ distance (later improve)
                if latency < best_latency:
                    best_latency = latency
            total_latency += best_latency
        return total_latency
    
    def update_position(self, current_pos, leader_pos, a):
        r1 = random.random()
        r2 = random.random()

        A = 2 * a * r1 - a
        C = 2 * r2

        D_leader = (abs(C * leader_pos[0] - current_pos[0]),
                    abs(C * leader_pos[1] - current_pos[1]),
                    abs(C * leader_pos[2] - current_pos[2]))

        X_leader = (
            leader_pos[0] - A * D_leader[0],
            leader_pos[1] - A * D_leader[1],
            leader_pos[2] - A * D_leader[2]
        )

        return X_leader

    def optimise(self):
        print("Running GWO Optimization...")

        # 1. Initialize wolf positions
        wolves = [uav.position for uav in self.uavs]

        # 2. Evaluate fitness
        fitness_scores = [self.fitness([wolf]) for wolf in wolves]  # evaluate each individually

        # 3. Identify alpha (best), beta (second), delta (third)
        sorted_wolves = [w for _, w in sorted(zip(fitness_scores, wolves))]
        alpha = sorted_wolves[0]
        beta = sorted_wolves[1]
        delta = sorted_wolves[2]

        # 4. Iterate
        for iteration in range(self.max_iter):
            a = 2 - (iteration * (2 / self.max_iter))  # linearly decrease 'a' from 2 to 0

            new_wolves = []
            for wolf in wolves:
                X1 = self.update_position(wolf, alpha, a)
                X2 = self.update_position(wolf, beta, a)
                X3 = self.update_position(wolf, delta, a)
                
                new_pos = (
                    (X1[0] + X2[0] + X3[0]) / 3,
                    (X1[1] + X2[1] + X3[1]) / 3,
                    (X1[2] + X2[2] + X3[2]) / 3
                )
                new_wolves.append(new_pos)

            wolves = new_wolves

            # Update alpha, beta, delta based on new fitness
            fitness_scores = [self.fitness([wolf]) for wolf in wolves]
            sorted_wolves = [w for _, w in sorted(zip(fitness_scores, wolves))]
            alpha = sorted_wolves[0]
            beta = sorted_wolves[1]
            delta = sorted_wolves[2]

            print(f"Iteration {iteration}: Best latency = {self.fitness([alpha])}")

        # 5. Update UAVs with new best positions
        for i, uav in enumerate(self.uavs):
            uav.move_to(wolves[i])

class PSO:
    def __init__(self, uavs, haps, request):
        self.uavs = uavs
        self.haps = haps
        self.request = request

    def optimise(self):
        # Placeholder for PSO Logic
        print("Running PSO Optimization...")
