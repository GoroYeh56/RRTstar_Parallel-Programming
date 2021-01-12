#!/bin/bash
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 1

# Plot RRT seq v.s. parallel
srun ./RRT 5000 3034 500 1 # 12 -c4 1500
# srun ./RRT 1000 3021 500 1 # 12 -c1 1500


# ----------- Strong scalability --------- 
# srun ./RRT 3000 3016 1000 2  # 12 -c2 500
# srun ./RRT 3000 3017 1000 2 # 12 -c3  1000
# srun ./RRT 3000 3018 500 1 # 12 -c4 1500

#  ----------- Try parallel for(K) RRT ------------------- #

#  ----------- Pthread Final version (from 1001) ------------------- #
# srun ./RRT 5000 2001 500 0 # FreeSpace, 500
# srun ./RRT 10000 2002 1000 0 # FreeSpace, 1000
# srun ./RRT 15000 2003 1500 0 # FreeSpace, 1500
# srun ./RRT 40000 2004 3000 0 # FreeSpace, 3000
# srun ./RRT 5000 2005 500 1 # FreeSpace, 500
# srun ./RRT 10000 2006 1000 1 # FreeSpace, 1000
# srun ./RRT 15000 2007 1500 1 # FreeSpace, 1500
# srun ./RRT 40000 2008 3000 1 # FreeSpace, 3000
# srun ./RRT 5000 2009 500 2 # FreeSpace, 500
# srun ./RRT 10000 2010 1000 2 # FreeSpace, 1000
# srun ./RRT 15000 2011 1500 2 # FreeSpace, 1500
# srun ./RRT 40000 2012 3000 2 # FreeSpace, 3000
# srun ./RRT 5000 2013 500 3 # FreeSpace, 500
# srun ./RRT 10000 2014 1000 3 # FreeSpace, 1000
# srun ./RRT 15000 2015 1500 3 # FreeSpace, 1500
# srun ./RRT 40000 2016 3000 3 # FreeSpace, 3000



# ------------ Sequential version -------------- #

# srun ./RRT 8000  81 500 0 # FreeSpace, 3000
# srun ./RRT 30000 82 1000 0 # FreeSpace, 1000
# srun ./RRT 40000 83 1500 0 # FreeSpace, 3000
# srun ./RRT 60000 84 3000 0 # FreeSpace, 1000

# srun ./RRT 8000 85 500 1 # 4-obs, 500
# srun ./RRT 30000 86 1000 1 # 4-obs, 1000
# srun ./RRT 40000 87 1500 1 # 4-obs, 1500
# srun ./RRT 60000 88 3000 1 # 4-obs, 3000


# srun ./RRT 8000 89 500 2 # 12-obs, 500
# srun ./RRT 30000 90 1000 2 # 12-obs, 1000
# srun ./RRT 40000 91 1500 2 # 12-obs, 1500
# srun ./RRT 60000 92 3000 2 # 12-obs, 3000


# srun ./RRT 8000 93 500 3 # maze, 500
# srun ./RRT 30000 94 1000 3 # maze, 1000
# srun ./RRT 40000 95 1500 3 # maze, 1500
# srun ./RRT 60000 96 3000 3 # maze, 3000


# srun ./RRT 5000 133 500 3 # maze, 500
# srun ./RRT 10000 134 1000 3 # maze, 1000
# srun ./RRT 15000 135 1500 3 # maze, 1500
# srun ./RRT 45000 136 3000 3 # maze, 3000


#  ----------- Parallel version ------------------- #
# srun ./RRT 8000 301 500 0 # FreeSpace, 500
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
# srun ./RRT 40000 126 3000 3 # maze 3000


# For debuggging: K = 10
# srun ./RRT 30 301 500 0 


#  ----------- Pthread version (from 401) ------------------- #

#  ----------- Pthread Final version (from 501) ------------------- #
# srun ./RRT 8000 801 500 0 # FreeSpace, 500
# srun ./RRT 30000 802 1000 0 # FreeSpace, 1000
# srun ./RRT 40000 803 1500 0 # FreeSpace, 1500
# srun ./RRT 60000 804 3000 0 # FreeSpace, 3000

# srun ./RRT 8000 805 500 1   # 4-obs 500
# srun ./RRT 30000 806 1000 1 # 4-obs 1000
# srun ./RRT 40000 807 1500 1 # 4-obs 1500
# srun ./RRT 60000 808 3000 1 # 4-obs 3000

# srun ./RRT 8000 609 500 2   # 12-obs 500
# srun ./RRT 30000 610 1000 2 # 12-obs 1000
# srun ./RRT 40000 611 1500 2 # 12-obs 1500
# srun ./RRT 60000 612 3000 2 # 12-obs 3000

# srun ./RRT 5000 613 500 3   # maze 500
# srun ./RRT 60000 614 1000 3 # maze 1000
# srun ./RRT 10000 615 1500 3 # maze 1500
# srun ./RRT 35000 416 3000 3 # maze 3000


# Variable : K

# srun ./RRT 1000 3001 500 0   # maze 500
# srun ./RRT 1500 3002 500 0 # maze 1000
# srun ./RRT 2000 3003 500 0 # maze 1500
# srun ./RRT 2500 3004 500 0 # maze 3000

# srun ./RRT 5000 3005 500 0   # maze 500
# srun ./RRT 5000 3006 500 1 # maze 1000
# srun ./RRT 5000 3007 500 2 # maze 1500
# srun ./RRT 5000 3008 500 3 # maze 3000

# Variable : size
# srun ./RRT 20000 3009 1500 0   # maze 500
# srun ./RRT 20000 3010 1500 0 # maze 1000
# srun ./RRT 20000 3011 1500 0 # maze 1500
# srun ./RRT 20000 3012 1500 0 # maze 3000
# Variable : size
# srun ./RRT 20000 3013 1500 3   # maze 500
# srun ./RRT 20000 3014 1500 3 # maze 1000
# srun ./RRT 20000 3015 1500 3 # maze 1500
# srun ./RRT 20000 3016 1500 3 # maze 3000





#  ----------- Remove findParent to test 'path' correctness(starting point) ------------------- #
# srun ./RRT 10000 201 500 0 # FreeSpace, 500
# srun ./RRT 30000 202 1000 0 # FreeSpace, 1000
# srun ./RRT 40000 203 1500 0 # FreeSpace, 1500
# srun ./RRT 60000 204 3000 0 # FreeSpace, 3000

# srun ./RRT 10000 115 500 1   # 4-obs 500
# srun ./RRT 30000 116 1000 1 # 4-obs 1000
# srun ./RRT 40000 117 1500 1 # 4-obs 1500
# srun ./RRT 60000 118 3000 1 # 4-obs 3000

# srun ./RRT 10000 119 500 2   # 12-obs 500
# srun ./RRT 30000 120 1000 2 # 12-obs 1000
# srun ./RRT 40000 121 1500 2 # 12-obs 1500
# srun ./RRT 60000 802 3000 2 # 12-obs 3000

# srun ./RRT 5000 123 500 3   # maze 500
# srun ./RRT 6000 124 1000 3 # maze 1000
# srun ./RRT 10000 125 1500 3 # maze 1500
# srun ./RRT 20000 127 3000 3 # maze 3000