import os
from pathlib import Path
import subprocess
import multiprocessing

# Define the array of util values with two decimal points
utils = ["0.40", "0.80", "1.20", "1.60", "2.00", "2.40", "2.80"]

# Define the base directory for the files
base_dir = "exp_9/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core/6-task/100-jitter"

# Function to run the command
def run_command(jobset_file, util):
    command = [
        "build/nptest",
        jobset_file,
        "-m", "4",
        "--search-based",
        "-f", "0.74,0.80,0.87,0.94,1.00",
        "--energy-timeout", "9000",
        "-o", "results/exp_1",
        "-u", util
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Command {command} failed with error: {e}")

# List to hold all job sets
job_sets = []

# Collect all job sets
for util in utils:
    jobset_dir = Path(base_dir) / f"{util}-util/jobsets"
    if jobset_dir.exists() and jobset_dir.is_dir():
        #print(f"Directory exists: {jobset_dir}")  # Debugging line
        for jobset_file in jobset_dir.iterdir():
            if jobset_file.is_file():
                job_sets.append((str(jobset_file), util))
            else:
                print(f"{jobset_file} is not a file.")  # Debugging line
    else:
        print(f"Directory {jobset_dir} does not exist.")  # Debugging line

# Function to process jobs using multiprocessing
def process_jobs():
    # Create a pool of worker processes
    with multiprocessing.Pool(processes=40) as pool:
        # Use starmap to pass multiple arguments to the run_command function
        pool.starmap(run_command, job_sets)

if __name__ == "__main__":
    #print(f"Current working directory: {os.getcwd()}")  # Debugging line
    process_jobs()