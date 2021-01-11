#!/bin/bash
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 1

srun ./RRT 70000 50 3000 0 # FreeSpace, 3000
# srun ./RRT 15000 32 1000 0 # FreeSpace, 1000

# Parallel version:
# srun ./RRT 8000 101 500 0 # FreeSpace, 500
# srun ./RRT 20000 102 1000 0 # FreeSpace, 1000
# srun ./RRT 50000 103 1500 0 # FreeSpace, 1500
# srun ./RRT 80000 104 3000 0 # FreeSpace, 3000

# srun ./RRT 8000 34 500 1 # 4-obs, 500
# srun ./RRT 20000 35 1000 1 # 4-obs, 1000
# srun ./RRT 50000 36 1500 1 # 4-obs, 1500

# Parallel version:
# srun ./RRT 8000 105 500 1   # 4-obs 500
# srun ./RRT 20000 106 1000 1 # 4-obs 1000
# srun ./RRT 50000 107 1500 1 # 4-obs 1500
# srun ./RRT 80000 108 3000 1 # 4-obs 3000


# srun ./RRT 8000 37 500 2 # 12-obs, 500
# srun ./RRT 50000 38 1000 2 # 12-obs, 1000
# srun ./RRT 50000 39 1500 2 # 12-obs, 1500

# Parallel version:
# srun ./RRT 8000 109 500 2   # 12-obs 500
# srun ./RRT 40000 110 1000 2 # 12-obs 1000
# srun ./RRT 50000 111 1500 2 # 12-obs 1500
# srun ./RRT 80000 112 3000 2 # 12-obs 3000



# srun ./RRT 8000 40 500 3 # maze, 500
# srun ./RRT 20000 41 1000 3 # maze, 1000
# srun ./RRT 50000 42 1500 3 # maze, 1500

# Parallel version:
# srun ./RRT 8000 113 500 3   # maze 500
# srun ./RRT 40000 114 1000 3 # maze 1000
# srun ./RRT 50000 115 1500 3 # maze 1500
# srun ./RRT 80000 116 3000 3 # maze 3000
