## Util for woking with Grids.


# Validate if one position in grid is valid.
def validate_index(ni, nj, grid):
	if(ni < 0 or ni >= len(grid)):
		return False
	if (nj < 0 or nj >= len(grid[0])):
		return False
	return True
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
		mvs = [[i - 1, j], [i,j+1], [i, j-1], [i+1,j]]

		for [ni,nj] in mvs:
			if validate_index(ni,nj,grid):
				# Not visited node
				if (D[ni][nj] < 0):
					D[ni][nj] = D[i][j] + 1									
					l.append([ni, nj])
					#TODO take into acount the other robots.
	# Return distance from robot to each cell
	return D
