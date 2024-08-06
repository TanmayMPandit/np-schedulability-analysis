import os
import pandas as pd
import numpy as np

# Define the ratios and the base directory path
ratios = [0.4, 0.5, 0.7, 0.8, 0.9, 1.0]
base_dir = 'exp_8/{}/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets'

# Iterate over each ratio directory
for ratio in ratios:
    # Create the full path to the jobsets directory for the current ratio
    dir_path = base_dir.format(ratio)
    
    # Check if the directory exists
    if os.path.exists(dir_path):
        # List all files in the directory
        for filename in os.listdir(dir_path):
            # Construct the full file path
            file_path = os.path.join(dir_path, filename)
            
            # Check if the current item is a file
            if os.path.isfile(file_path) and filename.endswith('.csv'):
                # Read the CSV file into a DataFrame
                df = pd.read_csv(file_path)
                
                # Update the 'Cost min' column
                df['Cost min'] = np.floor(ratio * df['Cost max']).astype(int)
                
                # Save the modified DataFrame back to the CSV file
                df.to_csv(file_path, index=False)
                
                # print(f"Updated file: {file_path}")
    else:
        print(f"Directory does not exist: {dir_path}")
