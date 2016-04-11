Header header

# Random number identifying this run (e.g. when the sender is restarted, this
# number changes)
uint32 runID

# ID of this keyframe (counter starting at 0)
uint32 keyframeID

# Compressed point cloud
uint8[] compressedCloud
