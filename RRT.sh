#!/bin/bash
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4

# ------------ Sequential version -------------- #

# srun ./RRT 8000  61 500 0 # FreeSpace, 3000
# srun ./RRT 20000 62 1000 0 # FreeSpace, 1000
# srun ./RRT 50000 63 1500 0 # FreeSpace, 3000
# srun ./RRT 80000 64 3000 0 # FreeSpace, 1000

# srun ./RRT 8000 65 500 1 # 4-obs, 500
# srun ./RRT 40000 66 1000 1 # 4-obs, 1000
# srun ./RRT 50000 67 1500 1 # 4-obs, 1500
# srun ./RRT 80000 68 3000 1 # 4-obs, 3000


# srun ./RRT 8000 69 500 2 # 12-obs, 500
# srun ./RRT 40000 70 1000 2 # 12-obs, 1000
# srun ./RRT 50000 71 1500 2 # 12-obs, 1500
# srun ./RRT 80000 72 3000 2 # 12-obs, 3000


# srun ./RRT 8000 73 500 3 # maze, 500
# srun ./RRT 40000 74 1000 3 # maze, 1000
# srun ./RRT 50000 75 1500 3 # maze, 1500
# srun ./RRT 80000 76 3000 3 # maze, 3000



#  ----------- Parallel version ------------------- #
# srun ./RRT 8000 111 500 0 # FreeSpace, 500
# srun ./RRT 20000 112 1000 0 # FreeSpace, 1000
# srun ./RRT 50000 113 1500 0 # FreeSpace, 1500
# srun ./RRT 80000 114 3000 0 # FreeSpace, 3000

# srun ./RRT 8000 115 500 1   # 4-obs 500
# srun ./RRT 20000 116 1000 1 # 4-obs 1000
# srun ./RRT 40000 117 1500 1 # 4-obs 1500
# srun ./RRT 50000 118 3000 1 # 4-obs 3000

# srun ./RRT 8000 119 500 2   # 12-obs 500
# srun ./RRT 40000 120 1000 2 # 12-obs 1000
# srun ./RRT 50000 121 1500 2 # 12-obs 1500
# srun ./RRT 80000 122 3000 2 # 12-obs 3000

# srun ./RRT 8000 123 500 3   # maze 500
# srun ./RRT 40000 124 1000 3 # maze 1000
# srun ./RRT 50000 125 1500 3 # maze 1500
# srun ./RRT 80000 126 3000 3 # maze 3000


# For debuggging: K = 10
srun ./RRT 30 301 500 0 

#  ----------- Remove findParent to test 'path' correctness(starting point) ------------------- #
# srun ./RRT 8000 201 500 0 # FreeSpace, 500
# srun ./RRT 20000 202 1000 0 # FreeSpace, 1000
# srun ./RRT 50000 203 1500 0 # FreeSpace, 1500
# srun ./RRT 60000 204 3000 0 # FreeSpace, 3000

# srun ./RRT 8000 115 500 1   # 4-obs 500
# srun ./RRT 20000 116 1000 1 # 4-obs 1000
# srun ./RRT 40000 117 1500 1 # 4-obs 1500
# srun ./RRT 50000 118 3000 1 # 4-obs 3000

# srun ./RRT 8000 119 500 2   # 12-obs 500
# srun ./RRT 40000 120 1000 2 # 12-obs 1000
# srun ./RRT 50000 121 1500 2 # 12-obs 1500
# srun ./RRT 80000 122 3000 2 # 12-obs 3000

# srun ./RRT 8000 123 500 3   # maze 500
# srun ./RRT 40000 124 1000 3 # maze 1000
# srun ./RRT 50000 125 1500 3 # maze 1500
# srun ./RRT 80000 126 3000 3 # maze 3000