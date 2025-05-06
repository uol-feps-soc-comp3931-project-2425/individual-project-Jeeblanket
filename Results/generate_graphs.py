import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load and label each experiment

static_df = pd.read_csv("Results/random_results_aggregated.csv")
static_df["Experiment"] = "Random"

optimised_df = pd.read_csv("Results/experiment_results_filtered.csv")
optimised_df["Experiment"] = "GWO BPSO"

greedy_df = pd.read_csv("Results/greedy_results_aggregated.csv")
greedy_df["Experiment"] = "Greedy"
# Combine all into one DataFrame
combined_df = pd.concat([static_df, optimised_df, greedy_df], ignore_index=True)

# Filter out rows with very large latency values (likely outliers or errors)
combined_df = combined_df[combined_df["avg_total_latency"] < 100]

# Set up subplots
fig, axs = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

# Plot: Latency vs R grouped by Experiment
sns.lineplot(data=combined_df, x="R", y="avg_total_latency", hue="Experiment", marker="o", ax=axs[0])
axs[0].set_title("Latency vs Arrival Rate (R)")
axs[0].set_xlabel("Arrival Rate (R)")
axs[0].set_ylabel("Average Latency (s)")
axs[0].grid(True)

# Plot: Latency vs C grouped by Experiment
sns.lineplot(data=combined_df, x="C", y="avg_total_latency", hue="Experiment", marker="o", ax=axs[1])
axs[1].set_title("Latency vs Max VNFs per UAV (C)")
axs[1].set_xlabel("Max VNFs per UAV (C)")
axs[1].set_ylabel("Average Latency (s)")
axs[1].grid(True)

# Adjust layout and save
plt.suptitle("Average Latency Comparisons by Experiment", fontsize=16)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig("latency_comp_by_experiment.png")
plt.show()