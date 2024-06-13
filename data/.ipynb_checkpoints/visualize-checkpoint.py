import json
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

# Specify the path to your .json file
json_file_path = '~/projects/ryo-os/data/data.json'

with open(json_file_path, 'r') as f:
    data = json.load(f)

# Extract time and weight data
time_data = data['time']
weight_data = data['weight']

# Convert time data to datetime objects
time_values = []
for time_point in time_data:
    time_point = datetime(1970, 1, 1) + timedelta(seconds=time_point['secs'], microseconds=time_point['nanos']//1000)
    time_values.append(time_point)

# Plot weight vs time
plt.figure(figsize=(10, 6))
plt.plot(time_values, weight_data)
plt.xlabel('Time')
plt.ylabel('Weight')
plt.title('Weight vs Time')
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
