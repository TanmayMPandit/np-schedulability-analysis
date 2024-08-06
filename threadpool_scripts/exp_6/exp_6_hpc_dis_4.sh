#!/bin/bash

#SBATCH --job-name="Exp_6:Distribution_4"
#SBATCH --partition=compute
#SBATCH --time=03:30:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=3G
#SBATCH --account=education-eemcs-msc-es

module load 2022r2
module load python/3.8.12

srun  python exp_6_dis_4.py