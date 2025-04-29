from math import exp
import random
import numpy as np
from classes import PARAMS, bandwidth, distance
import time

class GWO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.requests = requests
        self.max_iter = 100
        self.stagnation_threshold = 20
        self.mutation_interval = 20
        self.noise_strength = 0.05

    def bandwidth_vectorized(self, dists, link_type):
        if link_type == 'user_uav':
            bw_max = PARAMS["BW_max_user_uav"]
            r_max = PARAMS["R_v"]
        elif link_type == 'uav_hap':
            bw_max = PARAMS["BW_max_uav_hap"]
            r_max = PARAMS["R_h"]
        else:
            raise ValueError("Unknown link type")
        
        bw = bw_max * (1 - (dists / r_max))
        bw[dists > r_max] = 0
        return bw

    def fitness(self, uav_positions):
        hap_pos = np.array(self.haps[0].position)
        user_positions = np.array([req.user_position for req in self.requests])

        diff_user_uav = user_positions[:, np.newaxis, :] - np.array(uav_positions)[np.newaxis, :, :]
        dists_user_uav = np.linalg.norm(diff_user_uav, axis=2)

        dists_uav_hap = np.linalg.norm(np.array(uav_positions) - hap_pos, axis=1)
        dists_uav_hap = np.broadcast_to(dists_uav_hap, (len(self.requests), len(uav_positions)))

        bw_user_uav = self.bandwidth_vectorized(dists_user_uav, 'user_uav')
        bw_uav_hap = self.bandwidth_vectorized(dists_uav_hap, 'uav_hap')

        valid_links = (dists_user_uav <= PARAMS["R_v"]) & (dists_uav_hap <= PARAMS["R_h"]) & (bw_user_uav > 0) & (bw_uav_hap > 0)

        rcl = (PARAMS["latency_coeffs"]["alpha1"] * (PARAMS["S"] / bw_user_uav)) + \
              (PARAMS["latency_coeffs"]["alpha2"] * (PARAMS["S"] / bw_uav_hap))

        pl = PARAMS["latency_coeffs"]["gamma1"] * PARAMS["S_max"] + PARAMS["latency_coeffs"]["gamma2"] * (PARAMS["S"] / bw_uav_hap)
        prep = (PARAMS["latency_coeffs"]["beta1"] * (PARAMS["S"] / bw_uav_hap)) + PARAMS["latency_coeffs"]["beta2"]
        tx = PARAMS["latency_coeffs"]["delta1"] * (PARAMS["S"] / bw_user_uav)

        latency_matrix = rcl + pl + prep + tx
        latency_matrix[~valid_links] = np.inf

        best_latencies = np.min(latency_matrix, axis=1)
        best_latencies[np.isinf(best_latencies)] = 1e9

        return np.sum(best_latencies)

    def update_position(self, current_pos, leader_pos, a):
        r1, r2 = random.random(), random.random()
        A = 2 * a * r1 - a
        C = 2 * r2

        D_leader = np.abs(C * np.array(leader_pos) - np.array(current_pos))
        X_leader = np.array(leader_pos) - A * D_leader

        hap_pos = np.array(self.haps[0].position)
        dist_to_hap = np.linalg.norm(X_leader - hap_pos)

        if dist_to_hap > self.haps[0].communication_range:
            direction = hap_pos - X_leader
            X_leader = hap_pos - direction * (self.haps[0].communication_range / dist_to_hap)

        return tuple(X_leader)

    def optimise(self):
        print("GWO optimiser has begun")
        start_time = time.time()

        wolves = [uav.position for uav in self.uavs]
        fitness_scores = [self.fitness([wolf]) for wolf in wolves]

        sorted_pairs = sorted(zip(fitness_scores, wolves))
        fitness_scores_sorted, sorted_wolves = zip(*sorted_pairs)
        alpha, beta, delta = sorted_wolves[:3]
        best_latency_now = fitness_scores_sorted[0]

        prev_best_latency = best_latency_now
        stagnation_counter = 0
        self.best_latencies = [best_latency_now]

        for iteration in range(self.max_iter):
            a = 2 - (iteration * (2 / self.max_iter))

            new_wolves = []
            for wolf in wolves:
                X1 = self.update_position(wolf, alpha, a)
                X2 = self.update_position(wolf, beta, a)
                X3 = self.update_position(wolf, delta, a)

                new_pos = tuple(np.mean([X1, X2, X3], axis=0))
                new_wolves.append(new_pos)

            wolves = new_wolves

            # Add exploration kick every mutation_interval
            if iteration % self.mutation_interval == 0 and iteration != 0:
                wolves = [(pos[0] + np.random.uniform(-self.noise_strength, self.noise_strength),
                           pos[1] + np.random.uniform(-self.noise_strength, self.noise_strength),
                           pos[2] + np.random.uniform(-self.noise_strength, self.noise_strength)) for pos in wolves]

            fitness_scores = [self.fitness([wolf]) for wolf in wolves]
            sorted_pairs = sorted(zip(fitness_scores, wolves))
            fitness_scores_sorted, sorted_wolves = zip(*sorted_pairs)
            alpha, beta, delta = sorted_wolves[:3]
            best_latency_now = fitness_scores_sorted[0]

            self.best_latencies.append(best_latency_now)

            if abs(prev_best_latency - best_latency_now) < 1e-4:
                stagnation_counter += 1
            else:
                stagnation_counter = 0

            prev_best_latency = best_latency_now

            print(f"Iteration {iteration}: Best latency = {best_latency_now}")

            if stagnation_counter >= self.stagnation_threshold:
                print(f"Early stopping at iteration {iteration} due to stagnation.")
                break
        
        end_time = time.time()

        # Move UAVs to final positions
        for i, uav in enumerate(self.uavs):
            target_pos = wolves[i]
            move_distance = distance(uav.position, target_pos)
            if move_distance > uav.max_move:
                direction = np.array(target_pos) - np.array(uav.position)
                direction = direction * (uav.max_move / move_distance)
                new_pos = np.array(uav.position) + direction
                uav.move_to(tuple(new_pos))
            else:
                uav.move_to(target_pos)

            # Check if UAV can reach HAP
            reachable = False
            for hap in self.haps:
                if distance(uav.position, hap.position) <= hap.communication_range:
                    reachable = True
                    break
            uav.is_active = reachable

            # Enforce constraint 2.18 + 2.19: Limit active UAVs
            active_uavs = [uav for uav in self.uavs if uav.is_active]

            if len(active_uavs) > PARAMS["V_max"]:
                # Sort active UAVs by (number of connected users, then current load) ascending
                active_uavs.sort(key=lambda uav: (len(uav.connected_users), uav.current_load))

                # Deactivate excess UAVs
                excess = len(active_uavs) - PARAMS["V_max"]
                uavs_to_deactivate = active_uavs[:excess]

                for uav in uavs_to_deactivate:
                    uav.is_active = False

        return (end_time - start_time)

class PSO:
    def __init__(self, uavs, haps, requests):
        self.uavs = uavs
        self.haps = haps
        self.num_uavs = len(uavs)
        self.num_vnfs = 10
        self.max_iter = 100
        self.swarm_size = 50
        self.requests = requests

        # Parameters for better control
        self.logging_interval = 5
        self.stagnation_threshold = 25
        self.min_inertia = 0.4
        self.mutation_interval = 15

    def initialise_swarm(self):
        particles = []
        velocities = []
        for _ in range(self.swarm_size):
            particle = np.random.randint(0, 2, (self.num_uavs, self.num_vnfs))
            velocity = np.random.uniform(-1, 1, (self.num_uavs, self.num_vnfs))
            particles.append(particle)
            velocities.append(velocity)
        return particles, velocities

    def bandwidth_vectorized(self, dist, link_type):
        if link_type == 'user_uav':
            bw_max = PARAMS["BW_max_user_uav"]
            r_max = PARAMS["R_v"]
        elif link_type == 'uav_hap':
            bw_max = PARAMS["BW_max_uav_hap"]
            r_max = PARAMS["R_h"]
        else:
            raise ValueError("Unknown link type")

        bw = bw_max * (1 - (dist / r_max))
        bw[dist > r_max] = 0
        return bw

    def fitness(self, particle):
        uav_positions = np.array([uav.position for uav in self.uavs])
        user_positions = np.array([req.user_position for req in self.requests])
        num_requests = len(self.requests)
        hap_position = np.array(self.haps[0].position)

        diff_user_uav = user_positions[:, np.newaxis, :] - uav_positions[np.newaxis, :, :]
        dists_user_uav = np.linalg.norm(diff_user_uav, axis=2)

        dists_uav_hap = np.linalg.norm(uav_positions - hap_position, axis=1)
        dists_uav_hap = np.broadcast_to(dists_uav_hap, (num_requests, self.num_uavs))

        bw_user_uav = self.bandwidth_vectorized(dists_user_uav, link_type='user_uav')
        bw_uav_hap = self.bandwidth_vectorized(dists_uav_hap, link_type='uav_hap')

        particle_vnfs = particle

        vnfs_needed = np.zeros((num_requests, self.num_vnfs))
        for idx, req in enumerate(self.requests):
            vnfs_needed[idx, req.requested_vnfs] = 1

        vnfs_coverage = np.all((vnfs_needed[:, np.newaxis, :] <= particle_vnfs[np.newaxis, :, :]), axis=2)
        valid_links = (dists_user_uav <= PARAMS["R_v"]) & (dists_uav_hap <= PARAMS["R_h"]) & (bw_user_uav > 0) & (bw_uav_hap > 0)
        valid = vnfs_coverage & valid_links

        rcl = (PARAMS["latency_coeffs"]["alpha1"] * (dists_user_uav / bw_user_uav)) + \
              (PARAMS["latency_coeffs"]["alpha2"] * (dists_uav_hap / bw_uav_hap))

        rcl[~valid] = np.inf
        best_latencies = np.min(rcl, axis=1)
        best_latencies[np.isinf(best_latencies)] = 1e9

        return np.sum(best_latencies)

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def optimise(self):
        print("PSO optimiser has begun")
        start_time = time.time()

        stagnation_counter = 0

        old_activations = np.zeros((self.num_uavs, self.num_vnfs))
        for idx, uav in enumerate(self.uavs):
            for vnf in uav.active_vnfs:
                old_activations[idx, vnf] = 1

        particles, velocities = self.initialise_swarm()

        pbest = particles.copy()
        pbest_scores = [self.fitness(p) for p in pbest]

        gbest_idx = np.argmin(pbest_scores)
        gbest = pbest[gbest_idx]
        gbest_score = pbest_scores[gbest_idx]
        prev_best_score = gbest_score

        w = 0.9
        c1 = 1.5
        c2 = 1.3

        for iteration in range(self.max_iter):
            w = max(0.9 - (0.5 * iteration / self.max_iter), self.min_inertia)

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
                        prev_best_score = gbest_score
                        gbest = particles[i].copy()
                        gbest_score = score

            particle_std = np.std(np.stack(particles), axis=0)
            mean_particle_std = np.mean(particle_std)
            fitness_change = float(abs(prev_best_score - gbest_score))

            if fitness_change < 1e-3:
                stagnation_counter += 1
            else:
                stagnation_counter = 0

            if stagnation_counter >= self.stagnation_threshold:
                print(f"Early stopping at iteration {iteration} due to stagnation.")
                break

            if iteration % self.mutation_interval == 0 and iteration != 0:
                num_mutations = int(0.1 * self.swarm_size)
                mutation_indices = np.random.choice(self.swarm_size, num_mutations, replace=False)
                for idx in mutation_indices:
                    flip_rate = 0.1 if mean_particle_std < 0.05 else 0.05
                    flip = np.random.rand(self.num_uavs, self.num_vnfs) < flip_rate
                    particles[idx] = np.logical_xor(particles[idx], flip).astype(int)

            if iteration % self.logging_interval == 0:
                print(f"Iteration {iteration}: Swarm diversity = {mean_particle_std:.6f}")
                print(f"Iteration {iteration}: Fitness change = {fitness_change:.8f}")
                print(f"Iteration {iteration}: Best latency = {gbest_score}")

        end_time = time.time()

        # Finalize UAVs
        new_activations = np.zeros((self.num_uavs, self.num_vnfs))
        vnf_demand = np.zeros(self.num_vnfs)
        for request in self.requests:
            for vnf_id in request.requested_vnfs:
                vnf_demand[vnf_id] += 1

        for idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue
            needed_vnfs = set()
            for user in uav.connected_users:
                needed_vnfs.update(user.requested_vnfs)

            activated = np.where(gbest[idx] == 1)[0]
            valid_activated = [vnf for vnf in activated if vnf in needed_vnfs]

            # enforcing constraint 2.13
            if len(valid_activated) > uav.max_vnfs:
                valid_activated = random.sample(valid_activated, uav.max_vnfs)

            for vnf_id in valid_activated:
                new_activations[idx, vnf_id] = 1

        delta = new_activations - old_activations
        new_activations_count = np.sum(delta == 1)

        if new_activations_count > PARAMS["A_max"]:
            print(f"New activations ({new_activations_count}) exceed A_max ({PARAMS['A_max']}) - applying limit.")

            new_indices = np.argwhere(delta == 1)
            new_indices_sorted = sorted(new_indices, key=lambda x: vnf_demand[x[1]], reverse=True)

            allowed = new_indices_sorted[:PARAMS["A_max"]]
            delta[:] = 0
            for idx in allowed:
                delta[idx[0], idx[1]] = 1

            new_activations = old_activations + delta
            new_activations = np.clip(new_activations, 0, 1)

        for idx, uav in enumerate(self.uavs):
            if not uav.is_active:
                continue

            uav.active_vnfs.clear()
            vnf_indices = np.where(new_activations[idx] == 1)[0]
            for vnf_id in vnf_indices:
                uav.activate_vnf(vnf_id)

        print(f"Total optimization time: {end_time - start_time:.2f} seconds")
        return end_time - start_time