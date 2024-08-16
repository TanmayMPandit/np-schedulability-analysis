import os
from pathlib import Path
import subprocess
import multiprocessing

# Define the array of cores
cores = [
    ("10", "15")
]

# Number of cores to use
num_cores = 20

# Function to run the command
def run_command(jobset_file, core_value):
    command = [
        "build/nptest",
        jobset_file,
        "-m", core_value,
        "-f", "0.74,0.80,0.87,0.94,1.00",
        "--energy-timeout", "9000",
        "-o", "results/exp_7",
        "-u", core_value
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command {command} failed with error: {e}")

# List to hold all job sets
job_sets = []

# Loop through each core and task value
for core_value, task_value in cores:
    jobset_dir = f"exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/{core_value}-core/{task_value}-task/100-jitter/4.00-util/jobsets2"
    
    # Check if the directory exists
    jobset_path = Path(jobset_dir)
    if jobset_path.exists() and jobset_path.is_dir():
        for jobset_file in jobset_path.iterdir():
            if jobset_file.is_file():
                job_sets.append((str(jobset_file), core_value))
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
