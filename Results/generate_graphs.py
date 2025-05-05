import pandas as pd
import matplotlib.pyplot as plt

# Load the data
df = pd.read_csv("Results/experiment_results_final.csv")

# Remove rows with missing avg_total_latency
df = df.dropna(subset=["avg_total_latency"])

# Define parameters to plot against
params = {"U": "Number of UAVs", "R": "Requests per second", "C": "Maximum VNFs per UAV", "S_max": "Maximum UAV movement speed (m/s)", "V_max": "Maximum UAVs Active"}

# Set up plots
for param in params:
    grouped = df.groupby(param)["avg_total_latency"].mean().reset_index()

    plt.figure()
    plt.plot(grouped[param], grouped["avg_total_latency"], marker='o')
    plt.title(f"Average Latency vs {params[param]}")
    plt.xlabel(param)
    plt.ylabel("Average Total Latency")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"latency_vs_{param}.png")  # saves each plot as a PNG
    plt.show()