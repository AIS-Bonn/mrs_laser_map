Header header 

# Number of levels of the map
uint32 levels

# Size of map in meters
uint32 size_in_meters 

# Resolution of the map
float32 resolution 

# Number of points stored per cell
int32 cell_capacity

# Point cloud for each level 
sensor_msgs/PointCloud2[] cloud

# Free cell centers
geometry_msgs/Point[] free_cell_centers

bool distorted