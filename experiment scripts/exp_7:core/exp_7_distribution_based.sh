#!/bin/bash

# Define the array of cores
cores=(
    "2 3"
    "4 6"
    "6 9"
    "8 12"
    "10 15"
    "12 18"
)
# Construct the directory path. Select correct util value

    

# Loop through each util value
for core in "${cores[@]}"; do
    read core_value task_value <<< "$core"
    jobset_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/${core_value}-core/${task_value}-task/100-jitter/1.60-util/jobsets"
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # Update correct branching, link and search values
                (build/nptest "$jobset_file" -m $core_value -f "0.74,0.8,0.87,0.94,1.0"   --energy-timeout 3600 -o "results/exp_7" -u "$core_value") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait