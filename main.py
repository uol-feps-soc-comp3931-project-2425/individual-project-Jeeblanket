import SimPy
import math
import pandas as pd
from environment import SimulationEnvironment

def distance(pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)

def bandwidth(distance, link_type):
    """
    Calculates bandwidth based on distance and link type.

    Parameters:
    - distance (float): Distance between two nodes (meters)
    - link_type (str): 'user_uav' or 'uav_hap'

    Returns:
    - bandwidth (float): Calculated bandwidth (Mbps)
    """

    if link_type == 'user_uav':
        BW_max = 100   # Mbps for User <-> UAV
        R_max = 150000 # meters
    elif link_type == 'uav_hap':
        BW_max = 1000  # Mbps for UAV <-> HAP
        R_max = 400000 # meters
    else:
        raise ValueError(f"Unknown link type: {link_type}")

    # If distance is greater than max range, bandwidth is effectively 0
    if distance > R_max:
        return 1e-6  # or 0, but 1e-6 avoids division by zero issues later

    # Apply formula
    bandwidth = BW_max * (1 - (distance / R_max))
    return bandwidth

def main():
    # initialise environment, HAPS and UAVs
    s1 = SimulationEnvironment()
    
    # begin simulation
    s1.run_simulation()
    latency_df = pd.DataFrame(s1.latency_records)
    latency_df.to_csv("simulation_latency_results1.csv", index=False)

# 1. need a way to simulate:
#   a 3d spatial environment for UAV and HAP positions
#   communication ranges and network topology
#   user requests and mobility patterns
# 
# 2. Implement HAP and UAV classes
# need to model UAVs and HAPs as classes with properties such as:
#   position, range, assigned VNFs, current users
# Each UAV should have methods for checking user coverage, 
# activating/deactivating VNFs, repositioning
#
# 3. Model the request system
# simulate request arrivals with numpy.random.poisson or scipy.stats.poisson
# to simulate request arrivals
# need to also generate user locations and associate VNF needs
#
# 4. Implement PSO and GWO Algorithms
# keep them in seperate files
# use numpy for vector operations
# for bpso use sigmoid transformation + random threshold
# PySwarms is a library for PSO
#
# 5. Latency Model Implementation
# implement each latenct component as a function
# sum them in a total fitness function
#
# 6. Define Evaluation Experiments
# need to vary:
#   network size (number of HAPs/UAVs)
#   request density
#   compare baseline methods


