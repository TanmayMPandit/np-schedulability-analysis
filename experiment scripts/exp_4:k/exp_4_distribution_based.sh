#!/bin/bash

# Define the array of heuristics
ks=(2 3 4 5 6 7 8 9 10)
# Construct the directory path. Select correct util value
jobset_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"
    

# Loop through each util value
for k in "${ks[@]}"; do
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # Update correct branching and search values
                (build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" -k $k -b 0 --search_threshold 100   --energy-timeout 3600 -o "results/exp_4" -u "$k") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait