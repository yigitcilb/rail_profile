import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv("sensor_data.csv")

# Extract data
time = df["Time (s)"]
roll = df["Roll (deg)"]
road_taken = df["Road Taken"]

# Create subplots
fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# Plot Roll vs Time
axs[0].plot(time, roll, label="Roll (deg)", color="b")
axs[0].set_ylabel("Roll (deg)")
axs[0].legend()
axs[0].grid()

# Plot Road Taken vs Time
axs[1].plot(time, road_taken, label="Road Taken", color="r")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Road Taken")
axs[1].legend()
axs[1].grid()

# Show the plot
plt.tight_layout()
plt.show()
