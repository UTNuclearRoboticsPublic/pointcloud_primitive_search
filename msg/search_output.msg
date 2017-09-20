
# Right now, this is a kinda clunky way to re-store pcl_process.srv data inside another service file: primitive_process.srv

# pointcloud_process RESPONSE data
pointcloud_processing_server/pointcloud_task_result[] task_results
bool failed
int32 failed_during_task_num
