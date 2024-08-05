#!/bin/bash

# Define the array of heuristics
tasks=(5 11 12) 

# Loop through each util value
for task in "${tasks[@]}"; do
    # Construct the directory path. Select correct util value
    jobset_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/${task}-task/100-jitter/1.60-util/jobsets"
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # Update correct branching, link and search values
                (build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" --search-based   --energy-timeout 3600 -o "results/exp_6" -u "$task") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait