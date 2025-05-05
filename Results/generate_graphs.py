import pandas as pd
import matplotlib.pyplot as plt

# Load the data
df = pd.read_csv("Results/experiment_results_final.csv")

# Drop rows with missing latency
df = df.dropna(subset=["avg_total_latency"])

# Filter for specific parameters
filtered_df = df[
    (df["U"] == 100) &
    (df["C"] == 2) &
    (df["S_max"] == 60) &
    (df["V_max"] == 30)
]

# Group by R and collect latency values
grouped = [group["avg_total_latency"].values for _, group in filtered_df.groupby("R")]
labels = sorted(filtered_df["R"].unique())

# Create boxplot
plt.figure(figsize=(8, 5))
plt.boxplot(grouped, labels=labels, patch_artist=True)
plt.title("Latency vs Arrival Rate (R)\n(U=100, C=2, S_max=60, V_max=30)")
plt.xlabel("Arrival Rate (R)")
plt.ylabel("Average Total Latency (s)")
plt.grid(True)
plt.tight_layout()
plt.savefig("boxplot_latency_vs_R_filtered.png")
plt.show()