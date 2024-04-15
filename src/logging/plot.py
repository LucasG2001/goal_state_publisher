import os
import pandas as pd
import matplotlib.pyplot as plt

# Define timestamp
timestamp = "202404121519"

# Define file paths
folder_path = "data/" + timestamp + "/"
csv_files = ["action_primitive_"+timestamp,
             "ee_pose_"+timestamp,
             "hand_pose_"+timestamp,
             "nearest_point_"+timestamp,
             "reference_pose_"+timestamp]

# Create plots folder if it doesn't exist
plots_folder = "plots/" + timestamp + "/"
os.makedirs(plots_folder, exist_ok=True)

# Plot each CSV file
for file in csv_files:
    # Read CSV
    df = pd.read_csv(folder_path + file + ".csv")

    # Plot data
    plt.figure(figsize=(10, 6))
    plt.plot(df)
    plt.title(file)
    plt.xlabel("Time Step")
    plt.ylabel("Value")
    plt.grid(True)

    # Save plot
    plt.savefig(plots_folder + "data_" + file + ".png")
    # Optionally, save as jpg
    # plt.savefig(plots_folder + "data_" + file + ".jpg")

    # Close plot to avoid overlapping plots in memory
    plt.close()
