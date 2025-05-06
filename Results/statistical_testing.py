import pandas as pd
from scipy import stats
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# Load your datasets (same as before)
greedy_df = pd.read_csv("Results/greedy_results_aggregated.csv")
greedy_df["Experiment"] = "Greedy"

random_df = pd.read_csv("Results/random_results_aggregated.csv")
random_df["Experiment"] = "Random"

gwo_bpso_df = pd.read_csv("Results/experiment_results_filtered.csv")
gwo_bpso_df["Experiment"] = "GWO BPSO"

# Combine all into one DataFrame
combined_df = pd.concat([greedy_df, random_df, gwo_bpso_df], ignore_index=True)

# Optional: filter out extreme outliers
combined_df = combined_df[combined_df["dropped_requests"] < 5]

# One-way ANOVA
model = ols('dropped_requests ~ Experiment', data=combined_df).fit()
anova_table = sm.stats.anova_lm(model, typ=2)
print("ANOVA Results:\n", anova_table)

# Post-hoc test (Tukey HSD)
tukey = pairwise_tukeyhsd(endog=combined_df['dropped_requests'],
                          groups=combined_df['Experiment'],
                          alpha=0.05)
print("\nTukey HSD Post-Hoc Test:\n", tukey)