

#include "pointcloud_primitive_search/primitive_process_publisher.h"

PrimitiveProcessPublisher::PrimitiveProcessPublisher()
{
	
}

PrimitiveProcessPublisher::PrimitiveProcessPublisher(ros::NodeHandle nh, pointcloud_primitive_search::primitive_process process)
{
	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    //	ros::console::notifyLoggerLevelsChanged(); 
	nh_ = nh;
	this->updatePublishers(process);
}

void PrimitiveProcessPublisher::updatePublishers(pointcloud_primitive_search::primitive_process process)
{
	for(int i=0; i<process.request.inputs.size(); i++)
	{
		ros::Publisher clipping_pub = nh_.advertise<sensor_msgs::PointCloud2>(process.request.inputs[i].tasks[0].pub_topic, 1);
		ros::Publisher clipping_pub_less = nh_.advertise<sensor_msgs::PointCloud2>(process.request.inputs[i].tasks[0].pub_topic_remainder, 1);
		ros::Publisher primitive_pub = nh_.advertise<sensor_msgs::PointCloud2>(process.request.inputs[i].tasks[1].pub_topic, 1);
		ros::Publisher primitive_pub_less = nh_.advertise<sensor_msgs::PointCloud2>(process.request.inputs[i].tasks[1].pub_topic_remainder, 1);
		ROS_DEBUG_STREAM("[PrimitiveProcessPublisher] Created new publishers for primitive " << process.request.inputs[i].tasks[1].name << "; the " << i << "th primitive for this primitive_search.");
		clipping_pub_.push_back(clipping_pub);
		clipping_pub_less_.push_back(clipping_pub_less);
		primitive_pub_.push_back(primitive_pub);
		primitive_pub_less_.push_back(primitive_pub_less);

		ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>(process.request.inputs[i].tasks[1].pub_topic + "clipping_marker",1);
		marker_pub_.push_back(marker_pub);
	}
	is_initialized_ = true;
}

void PrimitiveProcessPublisher::publish(pointcloud_primitive_search::primitive_process process)
{
	if(is_initialized_)
	{
		for(int i=0; i<clipping_pub_.size(); i++)
		{
			marker_pub_[i].publish(process.request.inputs[i].clipping_marker);
			if(!process.response.outputs[i].failed || process.response.outputs[i].failed_during_task_num != 0) // Don't publish from failed tasks
			{
				// Don't publish unless cloud size > minimum 
				if(process.response.outputs[i].task_results[i].task_pointcloud.width*process.response.outputs[i].task_results[i].task_pointcloud.height > process.request.inputs[i].min_cloud_size)
				{
					ROS_DEBUG_STREAM("[PrimitiveProcessPublisher] Attempting to publish clipping cloud of size " << process.response.outputs[i].task_results[0].task_pointcloud.width*process.response.outputs[i].task_results[0].task_pointcloud.height << " from process " << process.request.inputs[i].tasks[0].name << ".");
					clipping_pub_[i].publish(process.response.outputs[i].task_results[0].task_pointcloud);
					clipping_pub_less_[i].publish(process.response.outputs[i].task_results[0].remaining_pointcloud);
				}
			}
			if(!process.response.outputs[i].failed) // Don't publish from failed tasks
			{
				// Don't publish unless cloud size > minimum 
				if(process.response.outputs[i].task_results[1].task_pointcloud.width*process.response.outputs[i].task_results[1].task_pointcloud.height > process.request.inputs[i].min_cloud_size)
				{
					ROS_DEBUG_STREAM("[PrimitiveProcessPublisher] Attempting to publish primitive primitive of size " << process.response.outputs[i].task_results[1].task_pointcloud.width*process.response.outputs[i].task_results[1].task_pointcloud.height << " from process " << process.request.inputs[i].tasks[1].name << ".");
					primitive_pub_[i].publish(process.response.outputs[i].task_results[1].task_pointcloud);
					primitive_pub_less_[i].publish(process.response.outputs[i].task_results[1].remaining_pointcloud);
				}
			}
		}
	}
}

void PrimitiveProcessPublisher::saveClouds(pointcloud_primitive_search::primitive_process process)
{
	for(int i=0; i<process.response.outputs.size(); i++)
	{
		if(!process.response.outputs[i].failed)
		{
			// Save ROS Bag File
			rosbag::Bag bag;
			std::string bag_name = "primitive_search_" + process.request.inputs[i].tasks[1].pub_topic + std::to_string(ros::Time::now().toSec()) + ".bag";
			bag.open(bag_name, rosbag::bagmode::Write);
			bag.write("/" + process.request.inputs[i].tasks[1].pub_topic, ros::Time::now(), process.response.outputs[i].task_results[1].task_pointcloud);
			ROS_INFO_STREAM("[PrimitiveSearch] Saved a ROSBAG to the file " << bag_name);
			// Save PCL PCD File 
			pcl::PointCloud<pcl::PointXYZI> temp_cloud;
			pcl::fromROSMsg(process.response.outputs[i].task_results[1].task_pointcloud, temp_cloud);
		    //pcl::io::savePCDFileASCII("tunnel_inspection_" + std::to_string(ros::Time::now().toSec()) + ".pcd", temp_cloud);
		    ROS_INFO_STREAM("[PrimitiveSearch] Saved " << temp_cloud.points.size() << " data points to a pcd file");
		}
	}
}

bool PrimitiveProcessPublisher::isInitialized()
{
	return is_initialized_;
}