import os
from pathlib import Path
import subprocess
import multiprocessing

# Define the array of execution ratios
execution_ratios = [0.4, 0.5, 0.7, 0.8, 0.9, 1.0]

# Number of cores to use
num_cores = 40

# Function to run the command
def run_command(jobset_file, ratio):
    command = [
        "build/nptest",
        jobset_file,
        "-m", "4",
        "--search-based",
        "-f", "0.74,0.80,0.87,0.94,1.00",
        "--energy-timeout", "3600",
        "-o", "results/exp_8",
        "-u", str(ratio)
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command {command} failed with error: {e}")

# List to hold all job sets
job_sets = []

# Loop through each execution ratio
for ratio in execution_ratios:
    jobset_dir = f"exp_8/{ratio}/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"
    
    # Check if the directory exists
    jobset_path = Path(jobset_dir)
    if jobset_path.exists() and jobset_path.is_dir():
        for jobset_file in jobset_path.iterdir():
            if jobset_file.is_file():
                job_sets.append((str(jobset_file), ratio))
            else:
                print(f"{jobset_file} is not a file.")
    else:
        print(f"Directory {jobset_dir} does not exist.")

# Function to process jobs using multiprocessing
def process_jobs():
    with multiprocessing.Pool(processes=num_cores) as pool:
        pool.starmap(run_command, job_sets)

if __name__ == "__main__":
    # print(f"Current working directory: {os.getcwd()}")
    # print(f"Using {num_cores} cores for multiprocessing")
    process_jobs()
