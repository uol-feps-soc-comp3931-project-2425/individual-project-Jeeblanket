import SimPy
import pandas as pd
from environment import SimulationEnvironment


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


