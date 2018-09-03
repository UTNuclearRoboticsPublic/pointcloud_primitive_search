
#ifndef PRIMITIVE_SEARCH_H
#define PRIMITIVE_SEARCH_H

#include "pointcloud_primitive_search/primitive_process.h"
#include "pointcloud_processing_server/pointcloud_utilities.h"
#include "pointcloud_processing_server/pointcloud_process_publisher.h"

#include <pointcloud_subtraction/pointcloud_subtraction.h>

class PrimitiveSearch
{
public: 
	PrimitiveSearch();
	PrimitiveSearch(std::vector< std::vector<float> > expected_coefficients, std::string yaml_file_name);
	std::vector<pointcloud_processing_server::pointcloud_process> search(sensor_msgs::PointCloud2 input);
	bool primitiveSearch(pointcloud_primitive_search::primitive_process::Request &req, pointcloud_primitive_search::primitive_process::Response &res);

private:
	ros::NodeHandle nh_;
	
	float plane_max_it;
	float plane_dist_thresh;
	float cylinder_max_it;
	float cylinder_dist_thresh;
	float min_cloud_size_;

	ros::ServiceClient client_;
	ros::ServiceServer server_;

};

#endif // PRIMITIVE_SEARCH_H