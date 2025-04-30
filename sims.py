import pandas as pd
import itertools
from environment import SimulationEnvironment
from classes import PARAMS

def run_experiments():
    results = []

    # Define PARAMS you want to vary
    U_values = [100, 500]          # number of UAVs
    R_values = [1, 10]                # request arrival rates
    C_values = [2, 6]                  # max VNFs per UAV
    S_max_values = [30, 90]         # max UAV movement speed
    V_max_values = [10, 30]          # max active UAVs

    # Number of times to repeat each setting (for averaging)
    num_repeats = 10

    # Create all combinations
    all_combinations = list(itertools.product(U_values, R_values, C_values, S_max_values, V_max_values))

    print(f"Total simulations to run: {len(all_combinations) * num_repeats}")

    experiment_id = 0
    for (U, R, C, S_max, V_max) in all_combinations:
        for repeat in range(num_repeats):

            print(f"\nRunning experiment {experiment_id}: U={U}, R={R}, C={C}, S_max={S_max}, V_max={V_max} (repeat {repeat+1})")

            # Set the PARAMS dynamically
            PARAMS["U"] = U
            PARAMS["R"] = R
            PARAMS["C"] = C
            PARAMS["S_max"] = S_max
            PARAMS["V_max"] = V_max

            # Create environment
            env = SimulationEnvironment()

            # Run simulation
            env.run_simulation()

            # Collect latency results
            if env.latency_records:
                avg_total_latency = sum(record['total'] for record in env.latency_records) / len(env.latency_records)
                avg_total_no_placement = sum(record['total_no_placement'] for record in env.latency_records) / len(env.latency_records)
                dropped_requests = sum(1 for record in env.latency_records if record['total'] >= 1e9)
                success_requests = len(env.latency_records) - dropped_requests
            else:
                avg_total_latency = None
                avg_total_no_placement = None
                dropped_requests = None
                success_requests = None

            results.append({
                'experiment_id': experiment_id,
                'U': U,
                'R': R,
                'C': C,
                'S_max': S_max,
                'V_max': V_max,
                'repeat': repeat + 1,
                'avg_total_latency': avg_total_latency,
                'avg_total_no_placement': avg_total_no_placement,
                'dropped_requests': dropped_requests,
                'successfully_served_requests': success_requests
            })

            experiment_id += 1

    # Save to CSV
    df = pd.DataFrame(results)
    df.to_csv('greedy_results_final.csv', index=False)
    print("\n All experiments completed! Results saved to 'experiment_results.csv'.")

if __name__ == "__main__":
    run_experiments()