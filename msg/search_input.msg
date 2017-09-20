
# pointcloud_process REQUEST data
sensor_msgs/PointCloud2 pointclouds
pointcloud_processing_server/pointcloud_task[] tasks
int32 min_cloud_size

float32[] expected_coefficients
float32[] clipping_boundaries

float32 angle_threshold
float32 offset_threshold
float32 radius_threshold
bool check_orientations
bool check_distances
bool check_radii

visualization_msgs/Marker clipping_marker