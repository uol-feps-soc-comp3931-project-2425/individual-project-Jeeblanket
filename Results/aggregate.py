import pandas as pd

# Load the CSV file containing repeated experiments
df = pd.read_csv("Results/static_results_final.csv")

# Drop rows where latency is missing
df = df.dropna(subset=["avg_total_latency"])

# Drop the 'repeat' column if it exists
if "repeat" in df.columns:
    df = df.drop(columns=["repeat"])

# Group by all experiment-defining parameters and calculate mean of numeric columns
grouped = df.groupby(["U", "R", "C", "S_max", "V_max"]).mean().reset_index()

# Save the aggregated results
grouped.to_csv("Results/static_results_aggregated.csv", index=False)

print("Aggregation complete. Output saved to 'experiment_results_aggregated.csv'.")
