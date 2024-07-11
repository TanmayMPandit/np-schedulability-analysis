#!/bin/bash

# Define the multiplied utilization values
utils=(0.4 0.8 1.2 1.6 2.0 2.4 2.8)

# Loop over each utilization value
for util in "${utils[@]}"; do
  # Convert the utilization value to the required format for the directory name
  util_formatted=$(printf "%.2f-util" "$util")

  # Loop over each jobset index from 0 to 9
  for jobset_index in {0..9}; do
    # Construct the jobset file name
    jobset_file="5_task_4_core/10-percent-jitter/${util_formatted}/jobsets/jobset-uniform-discrete_${jobset_index}.csv"

    # Execute the command
    output=$(build/nptest "$jobset_file" -m 4 -f "0.6,0.7,0.8,0.9,1.0")
    
    # Print the output
    echo "$output"
  done
done
