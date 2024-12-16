import pandas as pd
import numpy as np

# Load the CSV file
file_path = '/home/jeongwoo/Automotive_software_final/resources/csv/evaluation_lane/Lane_0.csv'  # Replace with your actual file path
lane_data = pd.read_csv(file_path)

# Extract x and y coordinates
x = lane_data['LanePointX'].to_numpy()
y = lane_data['LanePointY'].to_numpy()

# Define parameters
chunk_size = 100  # Number of points per sample
step_size = len(x) // 10  # Step size based on size / 10

# List to store a3 coefficients
a3_values = []

# Loop through the data using calculated step size
for i in range(0, len(x), step_size):
    # Ensure we don't exceed the available range
    if i + chunk_size > len(x):
        break
    
    # Get the current sample
    x_chunk = x[i:i + chunk_size]
    y_chunk = y[i:i + chunk_size]
    
    # Construct the design matrix for cubic polynomial fitting
    X = np.vstack([x_chunk**3, x_chunk**2, x_chunk, np.ones_like(x_chunk)]).T
    
    # Use pseudo-inverse to find the coefficients
    coefficients = np.linalg.pinv(X) @ y_chunk
    
    # Append the a3 coefficient
    a3_values.append(coefficients[0])

# Find the maximum value of a3
max_a3 = max(a3_values)
min_a3 = min(a3_values)

# Print the maximum a3 value
print("Maximum a3 coefficient:", max_a3)
print("Minimum a3 coefficient:", min_a3)
