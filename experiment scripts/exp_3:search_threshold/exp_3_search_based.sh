#!/bin/bash

# Define the array of search limit
search_limits=(10 20 30 40 50 60 70 80 90 100 110 120 130 140 150)
# Construct the directory path : Assign selected util
jobset_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"
    

# Loop through each util value
for limit in "${search_limits[@]}"; do
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # Set branching threshold that is selected
                (build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" --search-based   --search_threshold $limit  --energy-timeout 3600 -o "results/exp_3" -u "$limit") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait