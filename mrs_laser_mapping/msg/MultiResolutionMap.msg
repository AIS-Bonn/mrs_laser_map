Header header 

# Number of levels of the map
uint8 levels

# Size of map in meters
uint8 size_in_meters 

# Resolution of the map
uint8 resolution 

# Number of points stored per cell
int32 cell_capacity

# Point cloud for each level 
sensor_msgs/PointCloud2[] cloud

# Free cell centers
geometry_msgs/Point[] free_cell_centers

