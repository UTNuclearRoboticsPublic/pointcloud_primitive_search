
#ifndef PRIMITIVE_PROCESS_PUBLISHER_H
#define PRIMITIVE_PROCESS_PUBLISHER_H

#include <ros/ros.h>
#include "pointcloud_primitive_search/primitive_process.h"

class PrimitiveProcessPublisher
{
public:
	PrimitiveProcessPublisher();
	PrimitiveProcessPublisher(ros::NodeHandle nh, pointcloud_primitive_search::primitive_process process);
	void updatePublishers(pointcloud_primitive_search::primitive_process process);
	void publish(pointcloud_primitive_search::primitive_process);
	void createBag(std::string file_name);
	bool isInitialized();

private:
	std::vector<ros::Publisher> clipping_pub_;
	std::vector<ros::Publisher> clipping_pub_less_;
	std::vector<ros::Publisher> primitive_pub_;
	std::vector<ros::Publisher> primitive_pub_less_; 
	std::vector<ros::Publisher> marker_pub_;

	ros::NodeHandle nh_;
	bool is_initialized_;
};
#endif //PRIMITIVE_PROCESS_PUBLISHER_H