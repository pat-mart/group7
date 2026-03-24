#!/bin/bash
#SBATCH -c 1
#SBATCH --ntasks 3
#SBATCH -N 1
#SBATCH --mem=32G
#SBATCH -p cpu
#SBATCH -t 03:00:00
#SBATCH -o slurm-%j.out

make clean 
make 
./app
