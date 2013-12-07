#######################################
### Utilities for woking with Grids. ##
#######################################
from roomba_sensor.map import *
from math import *

# Validate if one position in grid is valid.
def validate_index(ni, nj, grid):
	if(ni < 0 or ni >= len(grid)):
		return False
	if (nj < 0 or nj >= len(grid[0])):
		return False
	return True


def maximum_neightbor(i, j, grid):
	mi, mj = -float("inf"), -float("inf")
	max = -float("inf")

	mvs = [[i - 1, j], [i, j+1], [i, j-1], 
		[i+1, j], [i + 1, j + 1], [i + 1, j-1], [i-1, j+1], [i - 1, j - 1]]

	for [ni,nj] in mvs:
		if not validate_index(ni,nj,grid):
			continue

		if grid[ni][nj] > max:
			mi, mj = ni, nj
			max = grid[ni][nj]
	return mi, mj


# BFS from position [spi, spj]	
def bread_first_search(spi, spj, grid):
	gn = len(grid)
	gm = len(grid[0])

	# l: Nodes for visiting
	# Add the current position
	l = [[spi, spj]]

	# Distance matrix
	D = [[-1 for i in xrange(gm)] for j in xrange(gn)]
	D[spi][spj] = 0	
	
	
	while (len(l) > 0):	
		# Extract in FIFO.	
		[i,j] = l.pop(0)
		# Possible movements[up, right, left, down]
		mvs = [[i - 1, j], [i, j+1], [i, j-1], [i+1, j]]

		for [ni,nj] in mvs:
			if validate_index(ni,nj,grid):
				# Not visited node
				if (D[ni][nj] < 0):
					D[ni][nj] = D[i][j] + 1									
					l.append([ni, nj])		

		# Diagonal movements
		mvs = [[i + 1, j + 1], [i + 1, j-1], [i-1, j+1], [i - 1, j - 1]]		
		for [ni,nj] in mvs:
			if validate_index(ni,nj,grid):
				# Not visited node
				if (D[ni][nj] < 0):
					D[ni][nj] = D[i][j] + sqrt(2)									
					l.append([ni, nj])

	# Return distance from robot to each cell
	return D


# Convert coordinates to grid position.
# (it uses constants from roomba_sensor.map ).
def coords_to_grid(x, y):
	spi = int((y - mapY1) / gdy)
	spj = int((x - mapX1) / gdx)
		
	if(spi > gm - 1):
		spi = gm - 1
	if(spj > gn - 1):
		spj = gn - 1
	if(spi < 0):
		spi = 0
	if (spj < 0):
		spj = 0
	return spi,spj

def grid_to_coords(i,j):
	x =  mapX1 + gdx * j + gdx / 2
	y =  mapY1 + gdy * i + gdy / 2
	return x, y