import os
from pathlib import Path
import subprocess
import multiprocessing

# Define the array of exploration limits
explore_limits = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

# Construct the directory path: Select correct util value
jobset_dir = "exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter/1.60-util/jobsets"

# Number of cores to use
num_cores = 40

# Function to run the command
def run_command(jobset_file, limit):
    command = [
        "build/nptest",
        jobset_file,
        "-m", "4",
        "-f", "0.74,0.80,0.87,0.94,1.00",
        "--link_threshold", str(limit),
        "--energy-timeout", "9000",
        "-o", "results/exp_5",
        "-u", str(limit)
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command {command} failed with error: {e}")

# List to hold all job sets
job_sets = []

# Check if the jobset directory exists
jobset_path = Path(jobset_dir)
if jobset_path.exists() and jobset_path.is_dir():
    for limit in explore_limits:
        for jobset_file in jobset_path.iterdir():
            if jobset_file.is_file():
                job_sets.append((str(jobset_file), limit))
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
