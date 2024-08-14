import os
from pathlib import Path
import subprocess
import multiprocessing

# Define the array of tasks
tasks = [12]

# Number of cores to use
num_cores = 30

# Function to run the command
def run_command(jobset_file, task):
    command = [
        "build/nptest",
        jobset_file,
        "-m", "4",
        "-f", "1.00",
        "--energy-timeout", "9000",
        "-o", "results/exp_6",
        "-u", str(task)
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command {command} failed with error: {e}")

# List to hold all job sets
job_sets = []

# Loop through each task value
for task in tasks:
    # Construct the directory path
    jobset_dir = f"exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/{task}-task/100-jitter/1.60-util/jobsets"
    
    # Check if the directory exists
    jobset_path = Path(jobset_dir)
    if jobset_path.exists() and jobset_path.is_dir():
        for jobset_file in jobset_path.iterdir():
            if jobset_file.is_file():
                job_sets.append((str(jobset_file), task))
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
