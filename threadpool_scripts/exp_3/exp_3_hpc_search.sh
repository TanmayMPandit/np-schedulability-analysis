#!/bin/bash

#SBATCH --job-name="Exp_3:Search"
#SBATCH --partition=compute
#SBATCH --time=02:30:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=2G
#SBATCH --account=education-eemcs-msc-es

module load 2022r2
module load python/3.8.12

srun  python exp_3_search.py