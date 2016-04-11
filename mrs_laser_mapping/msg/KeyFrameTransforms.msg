Header header

# Random number identifying this run (e.g. when the sender is restarted, this
# number changes)
uint32 runID

# Transform of the keyframe relative to /world_corrected_slam
geometry_msgs/Transform[] transforms

# start and end points of the graph edges (not same size as keyframe transforms)
geometry_msgs/Point[] edgeStartPoints
geometry_msgs/Point[] edgeEndPoints
