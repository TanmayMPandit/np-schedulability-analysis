#!/bin/bash

#SBATCH --job-name="test util_2.4"
#SBATCH --partition=compute
#SBATCH --time=00:30:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=1G
#SBATCH --account=education-eemcs-msc-es


srun test.sh