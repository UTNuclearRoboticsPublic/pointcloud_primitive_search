
#ifndef PRIMITIVE_PROCESS_CREATION_H
#define PRIMITIVE_PROCESS_CREATION_H

#include <ros/ros.h>
#include "pointcloud_primitive_search/primitive_process.h"
#include <pointcloud_processing_server/pointcloud_process.h>
#include <pointcloud_processing_server/pointcloud_task_creation.h>
#include <pointcloud_processing_server/pointcloud_utilities.h>

namespace PrimitiveProcessCreation
{
	void createProcesses(std::string yaml_file_name, pointcloud_primitive_search::primitive_process& primitive_process);
	void update_map(std::vector<float> map_offset, pointcloud_primitive_search::primitive_process& primitive_process);
	std::vector<float> plane_offset_values(std::vector<float> expected_coefficients, std::vector<float> found_coefficients);
	std::vector<float> cylinder_offset_values(std::vector<float> expected_coefficients, std::vector<float> found_coefficients);
};

#endif //PRIMITIVE_PROCESS_CREATION_H