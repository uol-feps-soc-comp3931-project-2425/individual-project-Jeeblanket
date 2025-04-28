import pandas as pd
from environment import SimulationEnvironment

def main():
    # initialise environment, HAPS and UAVs
    print("Beginning simulation...")
    s1 = SimulationEnvironment()
    
    # begin simulation
    s1.run_simulation()
    latency_df = pd.DataFrame(s1.latency_records)
    latency_df.to_csv("simulation_latency_results1.csv", index=False)

if __name__ == "__main__":
    main()
