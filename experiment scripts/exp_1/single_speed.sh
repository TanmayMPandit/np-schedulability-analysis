#!/bin/bash

# Define the array of util values
utils=(0.40 0.80 1.20 1.60 2.00 2.40 2.80)

# Define the base directory for the files
base_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter"

# Loop through each util value
for util in "${utils[@]}"; do
    # Construct the directory path
    jobset_dir="${base_dir}/${util}-util/jobsets"
    
    # Check if the directory exists
    if [[ -d "$jobset_dir" ]]; then
        # Loop through each file in the directory
        for jobset_file in "$jobset_dir"/*; do
            # Check if it is a file
            if [[ -f "$jobset_file" ]]; then
                # Run the command with the file
                # echo "Running: build/nptest \"$jobset_file\" -m 4 -f \"0.74,0.8,0.87,0.94,1.0\" -u \"$util\""
                (build/nptest "$jobset_file" -m 4 -f "1.0" -o "results/exp_1" -u "$util") &
            else
                # echo "$jobset_file is not a file."
            fi
        done
    else
        # echo "Directory $jobset_dir does not exist."
    fi
done

wait