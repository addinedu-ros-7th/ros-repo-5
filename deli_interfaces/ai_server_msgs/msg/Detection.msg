# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id

# 2D bounding box surrounding the object in pixels
ai_server_msgs/BoundingBox2D bbox

# segmentation mask of the detected object
# it is only the boundary of the segmented object
ai_server_msgs/Mask mask

# keypoints for human pose estimation
ai_server_msgs/KeyPoint2DArray keypoints