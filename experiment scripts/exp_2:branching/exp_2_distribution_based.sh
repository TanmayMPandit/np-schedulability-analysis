#!/bin/bash

# Define the array of heuristics
heuristics=(1 2 3 4)
# Construct the directory path
jobset_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"
    

# Loop through each util value
for heuristic in "${heuristics[@]}"; do
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # echo "Running: build/nptest \"$jobset_file\" -m 4 -f \"0.74,0.8,0.87,0.94,1.0\" -u \"$util\""
                (build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" -b $heuristic   --energy-timeout 3600 -o "results/exp_2" -u "$heuristic") &
            else
                echo "$jobset_file is not a file."
            fi
        done
    else
        echo "Directory $jobset_dir does not exist."
    fi
done

wait