#!/bin/bash

#SBATCH --job-name="test util_2.4"
#SBATCH --partition=compute
#SBATCH --time=00:30:00
#SBATCH --ntasks=25
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=2G
#SBATCH --account=education-eemcs-msc-es

# Define the multiplied utilization values
utils=(2.4)

# Loop over each utilization value
for util in "${utils[@]}"; do
  # Convert the utilization value to the required format for the directory name
  util_formatted=$(printf "%.2f-util" "$util")

  # Loop over each jobset index from 0 to 9
  for jobset_index in {0..99}; do
    # Construct the jobset file name
    jobset_file="100-jitter/${util_formatted}/jobsets/jobset-log-uniform-discrete_${jobset_index}.csv"

    # Execute the command
    srun build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" &
  done
done
wait