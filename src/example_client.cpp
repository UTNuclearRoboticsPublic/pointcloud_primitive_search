#include <ros/ros.h>
#include <pointcloud_processing_server/pointcloud_utilities.h>
#include <pointcloud_processing_server/pointcloud_process_publisher.h>
#include "pointcloud_primitive_search/primitive_process.h"
#include "pointcloud_primitive_search/primitive_process_creation.h"
#include "pointcloud_primitive_search/primitive_process_publisher.h"

class SegmentationClient
{

public:
  SegmentationClient();

private:
  void pointcloudCallback(const sensor_msgs::PointCloud2 pointcloud);

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;

  pointcloud_processing_server::pointcloud_process preprocess_;
  pointcloud_primitive_search::primitive_process primitive_process_;

  PointcloudProcessPublisher preprocessing_publisher_;

  bool service_has_worked_before_;
  bool shut_down_node_;

};

SegmentationClient::SegmentationClient()
{
  ROS_INFO("[PointcloudProcessingClient] Waiting for service server to start.");
  // wait for the service to start
  // CODE TO WAIT FOR SERVICE

  ROS_INFO("[PointcloudProcessingClient] Service server started.");  

  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/laser_stitcher/output_cloud", 5, &SegmentationClient::pointcloudCallback, this);
  std::string yaml_file_name;
  std::string primitive_yaml_file_name;
  nh_.param<std::string>("test_segmentation/base_yaml_file_name", yaml_file_name, "pointcloud_process");
  nh_.param<std::string>("test_segmentation/primitive_yaml_file_name", primitive_yaml_file_name, "primitive_search");

  PointcloudTaskCreation::processFromYAML(&preprocess_, yaml_file_name, "pointcloud_process");

  PrimitiveProcessCreation::createProcesses(&primitive_process_, primitive_yaml_file_name);

  service_has_worked_before_ = false;
  shut_down_node_ = false;

  preprocessing_publisher_.updateNodeHandle(nh_);
  preprocessing_publisher_.updatePublishers(preprocess_);

  for (int i=0; i<preprocess_.request.tasks.size(); i++)
  {
    preprocess_.request.tasks[i].should_publish = false;
    preprocess_.request.tasks[i].should_publish_remainder = false;
  }

  while(ros::ok() && !shut_down_node_)
  {
    ros::spinOnce();
  }
}

void SegmentationClient::pointcloudCallback(const sensor_msgs::PointCloud2 pointcloud_in)
{
  ros::Time time_callback_started = ros::Time::now();

  ROS_INFO("[PointcloudProcessingClient] Received pointcloud callback! Attempting service call...");
  preprocess_.request.pointcloud = pointcloud_in;

  ROS_INFO_STREAM(pointcloud_in.width*pointcloud_in.height << " " << preprocess_.request.tasks.size());

  ros::ServiceClient preprocess_client = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
  
  int num_times_failed = 0; 

  while ( !preprocess_client.call(preprocess_) ) // If we couldn't read output, the service probably isn't up yet --> retry
  {
    if(service_has_worked_before_)
    {
      ROS_ERROR_STREAM("[PointcloudProcessingClient] Service call failed, but has worked before - this likely means service node has crashed. Client node shutting down.");
      shut_down_node_ = true;
      return;
    }
    if(num_times_failed > 10)
    {
      ROS_ERROR_STREAM("[PointcloudProcessingClient] Service call has failed " << num_times_failed << " times. Shutting down client node.");
      shut_down_node_ = true;
      return;
    }
    ROS_INFO_STREAM("[PointcloudProcessingClient] Waiting for the 'pointcloud_service' service.");
    ros::Duration(1).sleep();
    num_times_failed++;
  }

  ROS_DEBUG_STREAM("[PointcloudProcessingClient] Finished basic processing. About to begin basic publishing phase...");
  preprocessing_publisher_.publish(preprocess_);

  ros::Duration first_process_duration = ros::Time::now() - time_callback_started;

  ROS_DEBUG_STREAM("[PointcloudProcessingClient] Initializing Primitive Search stuffs.");
  ros::ServiceClient search_client = nh_.serviceClient<pointcloud_primitive_search::primitive_process>("primitive_search");
  int basic_process_size = preprocess_.response.task_results.size();
  int last_type = preprocess_.request.tasks[basic_process_size-1].type_ind;
  if(last_type == 5 || last_type == 6 || last_type == 7)
    primitive_process_.request.pointcloud = preprocess_.response.task_results[basic_process_size-1].remaining_pointcloud;
  else
    primitive_process_.request.pointcloud = preprocess_.response.task_results[basic_process_size-1].task_pointcloud;
  PrimitiveProcessPublisher primitive_process_publisher(nh_, primitive_process_);

  ROS_DEBUG_STREAM("[PointcloudProcessingClient] About to begin Primitive Search part of processing!");
  while( !search_client.call(primitive_process_) )
  {
    ROS_ERROR_STREAM("[PointcloudProcessingClient] Attempt to call primitive search service failed - probably isn't up yet. Waiting and trying again...");
    ros::Duration(1.0).sleep();
  }
  primitive_process_publisher.publish(primitive_process_);

  service_has_worked_before_ = true; 

  ros::Duration primitive_search_duration = ros::Time::now() - time_callback_started - first_process_duration;
  ROS_INFO_STREAM("Basic Process Time: " << first_process_duration << "  Primitive Search Time: " << primitive_search_duration << "  Total Time: " << primitive_search_duration+first_process_duration);

  ros::Duration(.05).sleep();
}

int main (int argc, char **argv)
{ 
  ros::init(argc, argv, "test_segmentation");

  SegmentationClient seg_client;

  return 0;
}
