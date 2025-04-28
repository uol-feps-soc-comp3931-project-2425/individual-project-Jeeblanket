import SimPy
import math
import pandas as pd
from environment import SimulationEnvironment

def distance(pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)

def bandwidth(distance, link_type):
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

def __main__():
    # initialise environment, HAPS and UAVs
    s1 = SimulationEnvironment()
    
    # begin simulation
    s1.run_simulation()
    latency_df = pd.DataFrame(s1.latency_records)
    latency_df.to_csv("simulation_latency_results1.csv", index=False)
