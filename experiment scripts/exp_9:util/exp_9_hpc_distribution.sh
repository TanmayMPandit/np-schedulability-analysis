#!/bin/bash

#SBATCH --job-name="Exp_9:distribution"
#SBATCH --partition=compute
#SBATCH --time=01:30:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=1G
#SBATCH --account=education-eemcs-msc-es


srun exp_9_distribution_based.sh