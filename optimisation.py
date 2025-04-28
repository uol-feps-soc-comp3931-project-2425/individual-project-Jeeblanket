import math
import random
import numpy as np
from main import bandwidth, distance
from classes import PARAMS
import time

class GWO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.max_iter = 500
        self.requests = requests

    def fitness(self, uav_positions):
        # Calculate total latency based on UAV positions
        # (simplified estimated latency here)
        total_latency = 0

        hap = self.haps[0]

        for request in self.requests:
            best_latency = float('inf')
            for pos in uav_positions:
                # check distance from UAV to user
                dist_user_uav = distance(pos, request.user_position)
                # check distance form uav to hap
                dist_uav_hap = distance(pos, hap.position)
                # calculate bandwidths
                bw_user_uav = bandwidth(dist_user_uav, link_type='user_uav')
                bw_uav_hap = bandwidth(dist_uav_hap, link_type='uav_hap')
                # calculate request collection latency
                rcl = (PARAMS["latency_coeffs"]["alpha1"] * (dist_user_uav / bw_user_uav)) + (PARAMS["latency_coeffs"]["alpha2"] * (dist_uav_hap / bw_uav_hap))
                if rcl < best_latency:
                    best_latency = rcl

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
        start_time = time.time()

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
        
            # check if UAV can reach HAP
            # enforcing constraint 2.17
            reachable = False
            for hap in self.haps:
                d = distance(uav.position, hap.position)
                if d <= hap.communication_range:
                    reachable = True
                    break
            uav.is_active = reachable

        # enforce constraint 2.18
        # get all UAVs that are still active
        active_uavs = [uav for uav in self.uavs if uav.is_active]

        # If too many active UAVs, deactivate randomly
        if len(active_uavs) > PARAMS["V_max"]:
            # Sort UAVs by number of connected users (ascending),
            # and then by current load if there is a tie
            active_uavs.sort(key=lambda uav: (len(uav.connected_users), uav.current_load))
            # pick excess UAVs to deactivate (least useful first)
            excess = len(active_uavs) - PARAMS["V_max"]
            uavs_to_deactivate = active_uavs[:excess]

            for uav in uavs_to_deactivate:
                uav.is_active = False
        return (time.time() - start_time)

class PSO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.num_uavs = len(uavs)
        self.num_vnfs = 10
        self.max_iter = 500
        self.swarm_size = 30
        self.requests = requests

    def initialise_swarm(self):
        particles = []
        velocities = []
        for _ in range(self.swarm_size):
            # Random binary matrix for VNF activation (UAVs x VNFs)
            particle = np.random.randint(0, 2, (self.num_uavs, self.num_vnfs))
            velocity = np.random.uniform(-1, 1, (self.num_uavs, self.num_vnfs))
            particles.append(particle)
            velocities.append(velocity)
        return particles, velocities

    def fitness(self, particle):
        # Evaluate latency for this particle (VNF activation pattern)
        total_latency = 0

        hap = self.haps[0]

        for request in self.requests:
            best_latency = float('inf')
            for uav_idx, uav in enumerate(self.uavs):
                # UAV must have all VNFs requested
                vnfs_needed = set(request.requested_vnfs)
                vnfs_active = set(np.where(particle[uav_idx] == 1)[0])
                if not vnfs_needed.issubset(vnfs_active):
                    continue  # UAV cannot serve this request

                dist_user_uav = distance(uav.position, request.user_position)
                dist_uav_hap = distance(uav.position, hap.position)

                if dist_user_uav > PARAMS["R_v"] or dist_uav_hap > PARAMS["R_h"]:
                    continue  # Out of communication range

                bw_user_uav = bandwidth(dist_user_uav, link_type='user_uav')
                bw_uav_hap = bandwidth(dist_uav_hap, link_type='uav_hap')

                rcl = (PARAMS["latency_coeffs"]["alpha1"] * (dist_user_uav / bw_user_uav)) + \
                      (PARAMS["latency_coeffs"]["alpha2"] * (dist_uav_hap / bw_uav_hap))

                if rcl < best_latency:
                    best_latency = rcl

            if best_latency == float('inf'):
                best_latency = 1e9  # Big penalty if request cannot be served

            total_latency += best_latency

        return total_latency

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

    def optimise(self):

        old_activations = []
        for uav in self.uavs:
            # 1 if active, 0 if not
            state = np.zeros(self.num_vnfs)
            for vnf in uav.active_vnfs:
                state[vnf] = 1
            old_activations.append(state)
        old_activations = np.array(old_activations)

        start_time = time.time()

        particles, velocities = self.initialise_swarm()

        # Personal best
        pbest = particles.copy()
        pbest_scores = [self.fitness(p) for p in pbest]

        # Global best
        gbest_idx = np.argmin(pbest_scores)
        gbest = pbest[gbest_idx]
        gbest_score = pbest_scores[gbest_idx]

        w = 0.7  # inertia
        c1 = 1.5 # personal best weight
        c2 = 1.5 # global best weight

        for iteration in range(self.max_iter):
            for i in range(self.swarm_size):
                for u in range(self.num_uavs):
                    for v in range(self.num_vnfs):
                        r1 = random.random()
                        r2 = random.random()
                        velocities[i][u][v] = (w * velocities[i][u][v] +
                                               c1 * r1 * (pbest[i][u][v] - particles[i][u][v]) +
                                               c2 * r2 * (gbest[u][v] - particles[i][u][v]))

                        # Update particle using sigmoid + probability threshold
                        if random.random() < self.sigmoid(velocities[i][u][v]):
                            particles[i][u][v] = 1
                        else:
                            particles[i][u][v] = 0

                # Evaluate new fitness
                score = self.fitness(particles[i])

                # Update personal best
                if score < pbest_scores[i]:
                    pbest[i] = particles[i].copy()
                    pbest_scores[i] = score

                    # Update global best
                    if score < gbest_score:
                        gbest = particles[i].copy()
                        gbest_score = score

            print(f"Iteration {iteration}: Best latency = {gbest_score}")

        end_time = time.time()
        # 5. Update UAVs' active VNFs
        new_activations = []
        for uav_idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue # skip inactive UAVs

            active_vnfs = list(np.where(gbest[uav_idx] == 1)[0])

            # enforce max_vnfs limit (constraint 2.13)
            if len(active_vnfs) > uav.max_vnfs:
                active_vnfs = random.sample(active_vnfs, uav.max_vnfs) # if limit is exceeded, vnfs to activate are selected randomly

            # build new activation state
            new_state = np.zeros(self.num_vnfs)
            for vnf_id in active_vnfs:
                new_state[vnf_id] = 1
            new_activations.append(new_state)

        new_activations = np.array(new_activations)  # Shape (num_uavs, num_vnfs)

        delta = new_activations - old_activations  # Difference
        new_vnf_activations = np.sum(delta == 1)   # Count new activations

        # enforcing constraints 2.19 and 2.20
        if new_vnf_activations > PARAMS["A_max"]:
            print(f"Warning: {new_vnf_activations} new activations exceeds Amax ({PARAMS['A_max']}). Prioritizing important VNFs...")

            # build VNF demand count
            vnf_demand = np.zeros(self.num_vnfs)
            for request in self.requests:
                for vnf_id in request.requested_vnfs:
                    vnf_demand[vnf_id] += 1

            # find indices where new activations occurred
            new_activation_indices = np.argwhere(delta == 1)

            # sort by VNF demand (most important first)
            sorted_new_activations = sorted(new_activation_indices, 
                                    key=lambda idx: vnf_demand[idx[1]], 
                                    reverse=True)

            # keep only top Amax activations
            allowed_indices = sorted_new_activations[:PARAMS["A_max"]]

            # reset all
            delta[:, :] = 0

            # re activate only allowed ones
            for idx in allowed_indices:
                delta[idx[0], idx[1]] = 1

            new_activations = old_activations + delta
            new_activations = np.clip(new_activations, 0, 1)

        # finalize UAV updates
        for uav_idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue
            active_vnfs = list(np.where(new_activations[uav_idx] == 1)[0])
            uav.active_vnfs = set(active_vnfs)

        return end_time - start_time
