import os
import pandas as pd
import matplotlib.pyplot as plt

# Define timestamp
participant = "f_ext"
timestamp = "202406031646"
sampling_frequency = 20 # Hz

# Define file paths
folder_path = "data/" + participant + "/" + timestamp + "/"
csv_files = ["action_primitive_"+timestamp,
             "ee_pose_"+timestamp,
             "hand_pose_"+timestamp,
             "nearest_point_"+timestamp,
             "reference_pose_"+timestamp,
             "F_ext_"+timestamp]

# Create plots folder if it doesn't exist
plots_folder = "plots/" + participant + "/" + timestamp + "/"
os.makedirs(plots_folder, exist_ok=True)

# Plot each CSV file
for file in csv_files:
    # Read CSV
    df = pd.read_csv(folder_path + file + ".csv")
    time_values = df.index / sampling_frequency  # Assuming the sampling frequency is 30 Hz
    # Add the time column to the DataFrame
    df['time'] = time_values
    total_time = time_values[-1]  # Extract last timestep as total time

    # Plot data
    plt.figure(figsize=(10, 6))
    plt.plot(df['time'], df.drop(columns=['time']))  # Use 'time' as x-axis
    plt.title(file)
    plt.xlabel("Time (seconds)")
    plt.ylabel("Value")
    plt.grid(True)

    # Add total time to the plot as annotation
    plt.text(0.95, 0.05, f'Total Time: {total_time:.2f} s',
             verticalalignment='bottom', horizontalalignment='right',
             transform=plt.gca().transAxes,
             fontsize=10, bbox=dict(facecolor='white', alpha=0.5))

    # Save plot
    plt.savefig(plots_folder + "data_" + file + ".png")

    # Close plot to avoid overlapping plots in memory
    plt.close()
