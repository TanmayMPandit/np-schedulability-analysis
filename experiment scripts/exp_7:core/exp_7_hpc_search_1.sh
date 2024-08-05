#!/bin/bash

#SBATCH --job-name="Exp_7:search_1"
#SBATCH --partition=compute
#SBATCH --time=03:30:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=1G
#SBATCH --account=education-eemcs-msc-es


srun exp_7_search_based_1.sh