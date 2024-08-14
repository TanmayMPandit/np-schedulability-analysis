#!/bin/bash

#SBATCH --job-name="Exp_7:Distribution"
#SBATCH --partition=compute
#SBATCH --time=16:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=30
#SBATCH --mem=180G
#SBATCH --account=education-eemcs-msc-es

module load 2022r2
module load python/3.8.12

srun  python exp_7_dis.py