#!/bin/bash

# Defi-ne the array of heuristics
execution_ratios=(0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)
# Construct the directory path. Select correct util value

    

# Loop through each util value
for ratio in "${execution_ratios[@]}"; do
jobset_dir="${ratio}/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # Update correct branching, link and search values
                (build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" --search-based --link_threshold 50 -k 1 -b 0 --search_threshold 100   --energy-timeout 3600 -o "results/exp_8" -u "$ratio") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait