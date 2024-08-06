#!/bin/bash

#SBATCH --job-name="Exp_6:Single_3"
#SBATCH --partition=compute
#SBATCH --time=02:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=2G
#SBATCH --account=education-eemcs-msc-es

module load 2022r2
module load python/3.8.12

srun  python exp_6_single_3.py