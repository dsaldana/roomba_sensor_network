import rospy

# The map is represented by a rectangle from (x1,y1) to (x2,y2)
mapX1 = rospy.get_param('/map_x1', -5.0)
mapX2 = rospy.get_param('/map_x2', 5.0)
mapY1 = rospy.get_param('/map_y1', -5.0)
mapY2 = rospy.get_param('/map_y2', 5.0)
#Map size
mapLX = mapX2 - mapX1
mapLY = mapY2 - mapY1
# Grid size		
gn = rospy.get_param('/grid_n', 10) # Number of rows
gdx = mapLX / gn # delta x
gm = rospy.get_param('/grid_m', 10) # Number of columns
gdy = mapLY / gm # delta y
