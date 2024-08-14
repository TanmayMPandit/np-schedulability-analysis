import os
import subprocess
import multiprocessing
from pathlib import Path

output_dir = "2.8_5"
cores = 4

# Function to process each CSV file
def process_csv_file(csv_file):
    try:
        # Run the test command
        result = subprocess.run(
            ["build/nptest", csv_file, "-m", str(cores)],
            stdout=subprocess.PIPE,
            text=True,
            check=True
        ).stdout.strip()

        # Split the result into an array
        result_array = [item.strip() for item in result.split(',')]

        # Check if the test succeeded
        if result_array[1] == "1":
            print(f"Test succeeded")
        else:
            print(f"Test failed")
            os.remove(csv_file)

    except subprocess.CalledProcessError as e:
        print(f"Failed to run command on {csv_file}: {e}")

# Function to get all CSV files in the output directory and its subdirectories
def get_csv_files(directory):
    return list(Path(directory).rglob("jobset*.csv"))

# Main function to process all CSV files using multiprocessing
def main():
    csv_files = get_csv_files(output_dir)
    print(f"Found {len(csv_files)} CSV files to process.")

    with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
        pool.map(process_csv_file, csv_files)

if __name__ == "__main__":
    main()
