# The map is represented by a rectangle from (x1,y1) to (x2,y2)
mapX1 = -5.0
mapX2 = 5.0
mapY1 = -5.0
mapY2 = 5.0
#Map size
mapLX = mapX2 - mapX1
mapLY = mapY2 - mapY1
# Grid size		
gn = 10 # Number of rows
gdx = mapLX / gn # delta x
gm = 10 # Number of columns
gdy = mapLY / gm # delta y
