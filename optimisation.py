from math import exp
import random
import numpy as np
from classes import PARAMS, bandwidth, distance
import time

class GWO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.max_iter = 100
        self.requests = requests

    def fitness(self, uav_positions):
        total_latency = 0
        hap = self.haps[0]

        for request in self.requests:
            best_latency = float('inf')
            assigned = False

            for pos in uav_positions:
                # Calculate distance
                dist_user_uav = distance(pos, request.user_position)
                dist_uav_hap = distance(pos, hap.position)

                # Check if within range
                if dist_user_uav > PARAMS["R_v"] or dist_uav_hap > PARAMS["R_h"]:
                    total_latency+= 1e6 # Penalty if UAV is out of HAP range or user out of uav range

                # Calculate bandwidth
                bw_user_uav = bandwidth(dist_user_uav, link_type='user_uav')
                bw_uav_hap = bandwidth(dist_uav_hap, link_type='uav_hap')

                if bw_user_uav <= 0 or bw_uav_hap <= 0:
                    total_latency+= 1e6  # No usable link, penalty

                # Calculate latencies
                rcl = (PARAMS["latency_coeffs"]["alpha1"] * (PARAMS["S"] / bw_user_uav)) + \
                      (PARAMS["latency_coeffs"]["alpha2"] * (PARAMS["S"] / bw_uav_hap))
            
                pl = PARAMS["latency_coeffs"]["gamma1"] * PARAMS["S_max"] + \
                     PARAMS["latency_coeffs"]["gamma2"] * (PARAMS["S"] / bw_uav_hap)

                prep = (PARAMS["latency_coeffs"]["beta1"] * (PARAMS["S"] / bw_uav_hap)) + PARAMS["latency_coeffs"]["beta2"]

                tx = PARAMS["latency_coeffs"]["delta1"] * (PARAMS["S"] / bw_user_uav)

                latency = rcl + pl + prep + tx

                if latency < best_latency:
                    best_latency = latency
                    assigned = True

            if not assigned:
                # No UAV could serve this request
                total_latency += 1e9
            else:
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

        # check distance to HAP
        hap = self.haps[0]  # assume only 1 HAP
        dist_to_hap = distance(X_leader, hap.position)

        if dist_to_hap > hap.communication_range:
            # too far, pull back toward HAP
            dx = hap.position[0] - X_leader[0]
            dy = hap.position[1] - X_leader[1]
            dz = hap.position[2] - X_leader[2]

            scale = hap.communication_range / dist_to_hap  # shrink movement
            X_leader = (
                hap.position[0] - dx * scale,
                hap.position[1] - dy * scale,
                hap.position[2] - dz * scale
            )

        return X_leader

    def optimise(self):
        print("GWO optimiser has begun")
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
            d = distance(uav.position, wolves[i])
            
            # enforcing constraint 2.20
            # ensures the moce is phsycially possible within the timeframe
            # and if not, scales it down
            if d > uav.max_move:
                dx = wolves[i][0] - uav.position[0]
                dy = wolves[i][1] - uav.position[1]
                dz = wolves[i][2] - uav.position[2]

                scale = uav.max_move / d

                new_pos = (
                    wolves[i][0] + dx * scale,
                    wolves[i][1] + dy * scale,
                    wolves[i][2] + dz * scale
                )
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

        # enforce constraint 2.18 + 2.19
        # get all UAVs that are still active
        # active_uavs = [uav for uav in self.uavs if uav.is_active]

        # # If too many active UAVs, deactivate randomly
        # if len(active_uavs) > PARAMS["V_max"]:
        #     # Sort UAVs by number of connected users (ascending),
        #     # and then by current load if there is a tie
        #     active_uavs.sort(key=lambda uav: (len(uav.connected_users), uav.current_load))
        #     # pick excess UAVs to deactivate (least useful first)
        #     excess = len(active_uavs) - PARAMS["V_max"]
        #     uavs_to_deactivate = active_uavs[:excess]

        #     for uav in uavs_to_deactivate:
        #         uav.is_active = False

        print(time.time() - start_time)
        return (time.time() - start_time)

class PSO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.num_uavs = len(uavs)
        self.num_vnfs = 10
        self.max_iter = 100
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
        return 1 / (1 + np.exp(-x))

    def optimise(self):
        print("PSO optimiser has begun")
        start_time = time.time()

        # get old activation states (shape: num_uavs x num_vnfs)
        old_activations = np.zeros((self.num_uavs, self.num_vnfs))
        for idx, uav in enumerate(self.uavs):
            for vnf in uav.active_vnfs:
                old_activations[idx, vnf] = 1

        particles, velocities = self.initialise_swarm()

        # Personal best
        pbest = particles.copy()
        pbest_scores = [self.fitness(p) for p in pbest]

        # Global best
        gbest_idx = np.argmin(pbest_scores)
        gbest = pbest[gbest_idx]
        gbest_score = pbest_scores[gbest_idx]

        w = 0.9  # inertia
        c1 = 1.5 # personal best weight
        c2 = 2.0 # global best weight
    
        # PSO main loop
        for iteration in range(self.max_iter):
            for i in range(self.swarm_size):
                r1, r2 = np.random.rand(self.num_uavs, self.num_vnfs), np.random.rand(self.num_uavs, self.num_vnfs)
                velocities[i] = (w * velocities[i] +
                             c1 * r1 * (pbest[i] - particles[i]) +
                             c2 * r2 * (gbest - particles[i]))

                prob = self.sigmoid(velocities[i])
                random_matrix = np.random.rand(self.num_uavs, self.num_vnfs)
                particles[i] = (random_matrix < prob).astype(int)

                score = self.fitness(particles[i])

                if score < pbest_scores[i]:
                    pbest[i] = particles[i].copy()
                    pbest_scores[i] = score
                    if score < gbest_score:
                        gbest = particles[i].copy()
                        gbest_score = score
            if iteration % 10 == 0:
                w -= 0.05

            print(f"Iteration {iteration}: Best latency = {gbest_score}")

        end_time = time.time()

        # Now finalize UAV states
        new_activations = np.zeros((self.num_uavs, self.num_vnfs))

        # Build VNF demand once
        vnf_demand = np.zeros(self.num_vnfs)
        for request in self.requests:
            for vnf_id in request.requested_vnfs:
                vnf_demand[vnf_id] += 1

        for idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue

            # Find VNFs needed by users
            needed_vnfs = set()
            for user in uav.connected_users:
                needed_vnfs.update(user.requested_vnfs)

            activated = np.where(gbest[idx] == 1)[0]
            valid_activated = [vnf for vnf in activated if vnf in needed_vnfs]

            if len(valid_activated) > uav.max_vnfs:
                valid_activated = random.sample(valid_activated, uav.max_vnfs)

            for vnf_id in valid_activated:
                new_activations[idx, vnf_id] = 1

        # Constraint: A_max
        delta = new_activations - old_activations
        new_activations_count = np.sum(delta == 1)

        if new_activations_count > PARAMS["A_max"]:
            print(f" New activations ({new_activations_count}) exceed A_max ({PARAMS['A_max']}) - applying limit.")

            new_indices = np.argwhere(delta == 1)

            # prioritize new activations based on VNF demand
            new_indices_sorted = sorted(new_indices, key=lambda x: vnf_demand[x[1]], reverse=True)

            allowed = new_indices_sorted[:PARAMS["A_max"]]
            delta[:] = 0  # Reset all
            for idx in allowed:
                delta[idx[0], idx[1]] = 1

            new_activations = old_activations + delta
            new_activations = np.clip(new_activations, 0, 1)  # Keep 0/1

        for idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue

            # find VNFs that PSO wants active
            vnf_indices = np.where(new_activations[idx] == 1)[0]

            # clear old VNFs first (optional but recommended)
            uav.active_vnfs.clear()

            for vnf_id in vnf_indices:
                uav.activate_vnf(vnf_id)

        print(time.time() - start_time)
        return end_time - start_time
