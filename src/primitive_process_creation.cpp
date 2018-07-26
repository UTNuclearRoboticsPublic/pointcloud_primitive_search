
#include "pointcloud_primitive_search/primitive_process_creation.h"

namespace PrimitiveProcessCreation
{
	void createProcesses(pointcloud_primitive_search::primitive_process *primitive_process, std::string yaml_file_name)
	{
	// -------------------------------------------------------------------------------------------------
	// ------------------- Basic initializations of some necessary global parameters -------------------
	    // These values selected to be unrealistically large --> when threshold not specified, checks should evaluate to TRUE 
	    float default_angle_difference = 365;           // degrees
	    float default_threshold_distance = 10000;       // meters

	    // Get basic task information from yaml file --> later, split this into a list of processes each with one task 
	    pointcloud_processing_server::pointcloud_process all_task_process;
	    PointcloudTaskCreation::processFromYAML(&all_task_process, yaml_file_name, "primitive_search");

	    ROS_DEBUG_STREAM("[PrimitiveSearch] Call to initialize searchForPrimitives object received.");

	    ros::NodeHandle nh;
	    // Global parameters for segmentation preferences (across cylinders, planes...)
	    float plane_max_iterations, plane_threshold_dist, cylinder_max_iterations, cylinder_threshold_dist;        
	    if( nh.getParam("primitive_search/" + yaml_file_name + "/plane_options/max_iterations", plane_max_iterations) )
	        for(int i=0; i<all_task_process.request.tasks.size(); i++)
	            if(all_task_process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK)
	                all_task_process.request.tasks[i].parameters[0] = plane_max_iterations;
	    if( nh.getParam("primitive_search/" + yaml_file_name + "/plane_options/max_iterations", plane_threshold_dist) )
	        for(int i=0; i<all_task_process.request.tasks.size(); i++)
	            if(all_task_process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK)
	                all_task_process.request.tasks[i].parameters[0] = plane_threshold_dist;
	    if( nh.getParam("primitive_search/" + yaml_file_name + "/cylinder_options/max_iterations", cylinder_max_iterations) )
	        for(int i=0; i<all_task_process.request.tasks.size(); i++)
	            if(all_task_process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK)
	                all_task_process.request.tasks[i].parameters[0] = cylinder_max_iterations;
	    if( nh.getParam("primitive_search/" + yaml_file_name + "/cylinder_options/max_iterations", cylinder_threshold_dist) )
	        for(int i=0; i<all_task_process.request.tasks.size(); i++)
	            if(all_task_process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK)
	                all_task_process.request.tasks[i].parameters[0] = cylinder_threshold_dist;
	    std::vector<float> map_offset; 
	    if( !nh.getParam("primitive_search/" + yaml_file_name + "/map_offset", map_offset) )
	        for(int i=0; i<6; i++)
	            map_offset.push_back(0);
	// ------------------------------------------------------------------------------------------
	// --------------------- Create list of processes and associated data -----------------------

	    primitive_process->request.inputs.clear();
	    bool temp_bool;
	    float temp_float;
	    // Splitting the single primitive_process into a list of single-task processes, one for each primitive desired
	    for(int i=0; i<all_task_process.request.tasks.size(); i++)
	    {
	      // --------------------- Create Processes -----------------------
	        pointcloud_primitive_search::search_input search_input;
	        pointcloud_processing_server::pointcloud_task temp_task;
	        // --------------------- Create Clipping Task -----------------------
	        temp_task.name = all_task_process.request.tasks[i].name + "_clipping";
	        temp_task.type_ind = pointcloud_processing_server::pointcloud_task::CLIPPING_TASK;      
	        temp_task.keep_ordered = false;
	        temp_task.should_publish = all_task_process.request.tasks[i].should_publish;
	        temp_task.pub_topic = all_task_process.request.tasks[i].pub_topic + "_clipping";
	        temp_task.should_publish_remainder = all_task_process.request.tasks[i].should_publish_remainder;
	        temp_task.pub_topic_remainder = all_task_process.request.tasks[i].pub_topic_remainder + "_clipping";

	        search_input.tasks.push_back(temp_task);

	        // --------------------- Create Segmentation Task -----------------------
	        temp_task.name = all_task_process.request.tasks[i].name;
	        temp_task.type_ind = all_task_process.request.tasks[i].type_ind;
	        temp_task.parameters.push_back(all_task_process.request.tasks[i].parameters[0]);        
	        temp_task.parameters.push_back(all_task_process.request.tasks[i].parameters[1]);       
	        if(temp_task.type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK)     // Add max_radius (only for Cylinder Segmentation)
	             temp_task.parameters.push_back(all_task_process.request.tasks[i].parameters[2]);       
	        temp_task.should_publish = all_task_process.request.tasks[i].should_publish;
	        temp_task.pub_topic = all_task_process.request.tasks[i].pub_topic;
	        temp_task.should_publish_remainder = all_task_process.request.tasks[i].should_publish_remainder;
	        temp_task.pub_topic_remainder = all_task_process.request.tasks[i].pub_topic_remainder;
	        temp_task.remove_cloud = all_task_process.request.tasks[i].remove_cloud;
	        
	        search_input.tasks.push_back(temp_task);
	        search_input.min_cloud_size = 50;

	        // --------------------- Create Other Parameter Lists -----------------------
	        std::vector<float> temp_coefficients;
	        std::vector<float> temp_clipping_data;
	        // retrieve list of EXPECTED COEFFICIENTS
	        if( !nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/expected_coefficients", temp_coefficients) )
	        {
	            ROS_ERROR_STREAM("[PrimitiveSearch] Failed to get expected coefficients for task " << temp_task.name << "! Aborting search...");
	            return;
	        }
	        else
	        {
	            search_input.expected_coefficients = temp_coefficients;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added expected coefficients for segmentation task " << temp_task.name << ".");
	        }
	        // retrieve list of CLIPPING_BOUNDARIES
	        if( !nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/clip_boundaries", temp_clipping_data) )
	        {
	            ROS_ERROR_STREAM("[PrimitiveSearch] Failed to get clipping boundaries for task " << temp_task.name << "! Aborting search...");
	            return;
	        }
	        else
	        {
	            search_input.clipping_boundaries = temp_clipping_data;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added clipping boundaries for segmentation task " << temp_task.name << ".");
	        } 
	        // retrieve ANGLE DISTANCE THRESHOLD limit
	        if( !nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/angle_threshold", temp_float) )
	        {
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Failed to get angle threshold for task " << temp_task.name << "! Defaulting to " << default_angle_difference << ".");
	            search_input.angle_threshold = default_angle_difference;
	        }
	        else
	        {
	            search_input.angle_threshold = temp_float;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added angle threshold for segmentation task " << temp_task.name << ".");
	        }
	        // retrieve DISTANCE THRESHOLD limit
	        if( !nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/offset_threshold", temp_float) )
	        {
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Failed to get offset threshold for task " << temp_task.name << "! Defaulting to " << default_threshold_distance << ".");
	            search_input.offset_threshold = default_threshold_distance;
	        }
	        else
	        {
	            search_input.offset_threshold = temp_float;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added offset threshold for segmentation task " << temp_task.name << ".");
	        }
	        // retrieve RADIUS DIFFERENCE THRESHOLD limit 
	        if( !nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/radius_threshold", temp_float) )
	        {
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Failed to get radius threshold for task " << temp_task.name << "! Defaulting to " << default_threshold_distance << ".");
	            search_input.radius_threshold = default_threshold_distance;
	        }
	        else
	        {
	            search_input.radius_threshold = temp_float;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added radius threshold for segmentation task " << temp_task.name << ".");
	        }
	        // retrieve boolean need to CHECK CONSTRAINTS
	          // CHECK RADIUS
	        if( nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/check_radius", temp_bool) )
	        {
	            search_input.check_radii = temp_bool;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added radius check need for segmentation task " << temp_task.name << ".");
	        }
	        else
	            search_input.check_radii = false;       // this default will usually occur for planes (which have no radius)
	          // CHECK ORIENTATION
	        if( nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/check_orientation", temp_bool) )
	        {
	            search_input.check_orientations = temp_bool;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added orientation check need for segmentation task " << temp_task.name << ".");
	        }
	        else
	            search_input.check_orientations = false;
	          // CHECK DISTANCE 
	        if( nh.getParam("primitive_search/" + yaml_file_name + "/tasks/" + temp_task.name + "/check_distance", temp_bool) )
	        {
	            search_input.check_distances = temp_bool;
	            ROS_DEBUG_STREAM("[PrimitiveSearch] Added offset check need for segmentation task " << temp_task.name << ".");
	        }
	        else
	            search_input.check_distances = false;

	        // Assign CLIPPING BOX DIMENSIONS
	        for(int j=0; j<6; j++)
	            search_input.tasks[0].parameters.push_back(search_input.clipping_boundaries[j]);

	        geometry_msgs::Pose clipping_pose;
	        int marker_type;

	        if(search_input.tasks[1].type_ind == pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK)
	        {
	            clipping_pose = PointcloudUtilities::clippingBoundsPlane(search_input.expected_coefficients); 
	            marker_type = visualization_msgs::Marker::CUBE; 
	            search_input.expected_coefficients = PointcloudUtilities::offsetPlaneCoefficients(search_input.expected_coefficients, map_offset);
	        }
	        else if(search_input.tasks[1].type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK)
	        {
	            clipping_pose = PointcloudUtilities::clippingBoundsCylinder(search_input.expected_coefficients); 
	            marker_type = visualization_msgs::Marker::CUBE;             // Although the primitive is a cylinder, the clipping box is still a cube 
	            search_input.expected_coefficients = PointcloudUtilities::offsetCylinderCoefficients(search_input.expected_coefficients, map_offset);
	        }
	        clipping_pose.position.x -= map_offset[0];
	        clipping_pose.position.y -= map_offset[1];
	        clipping_pose.position.z -= map_offset[2];
	        //ROS_ERROR_STREAM("assigning clipping_pose to parameters");
	        search_input.tasks[0].parameters.push_back(clipping_pose.position.x);
	        search_input.tasks[0].parameters.push_back(clipping_pose.position.y);
	        search_input.tasks[0].parameters.push_back(clipping_pose.position.z);
	        // Convert quaternion message to RPY values: 
	        tf::Quaternion tf_quat;
	        tf::Matrix3x3 rpy_conversion_matrix;
	        tf::quaternionMsgToTF(clipping_pose.orientation, tf_quat);
	        rpy_conversion_matrix.setRotation(tf_quat);
	        double roll, pitch, yaw;
	        rpy_conversion_matrix.getRPY(roll, pitch, yaw);
	        roll -= map_offset[3];
	        pitch -= map_offset[4];
	        yaw -= map_offset[5];

	        tf_quat.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(tf_quat, clipping_pose.orientation);
	        // Assign CLIPPING BOX POSE
	        search_input.tasks[0].parameters.push_back(roll);
	        search_input.tasks[0].parameters.push_back(pitch);
	        search_input.tasks[0].parameters.push_back(yaw);

	        search_input.clipping_marker = PointcloudUtilities::makeClippingVisualization(clipping_pose, temp_clipping_data, search_input.tasks[0].name, i, marker_type);

	        primitive_process->request.inputs.push_back(search_input);

	    }
	}

	void update_map(std::vector<float> map_offset, pointcloud_primitive_search::primitive_process& primitive_process)
	{
		for(int i=0; i<primitive_process.request.inputs.size(); i++)
		{
			// Assign CLIPPING BOX DIMENSIONS
	        for(int j=0; j<6; j++)
	            primitive_process.request.inputs[i].tasks[0].parameters.push_back(primitive_process.request.inputs[i].clipping_boundaries[j]);

	        geometry_msgs::Pose clipping_pose;
	        int marker_type;

	        if(primitive_process.request.inputs[i].tasks[1].type_ind == pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK)
	        {
	            clipping_pose = PointcloudUtilities::clippingBoundsPlane(primitive_process.request.inputs[i].expected_coefficients); 
	            marker_type = visualization_msgs::Marker::CUBE; 
	            primitive_process.request.inputs[i].expected_coefficients = PointcloudUtilities::offsetPlaneCoefficients(primitive_process.request.inputs[i].expected_coefficients, map_offset);
	        }
	        else if(primitive_process.request.inputs[i].tasks[1].type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK)
	        {
	            clipping_pose = PointcloudUtilities::clippingBoundsCylinder(primitive_process.request.inputs[i].expected_coefficients); 
	            marker_type = visualization_msgs::Marker::CUBE;             // Although the primitive is a cylinder, the clipping box is still a cube 
	            primitive_process.request.inputs[i].expected_coefficients = PointcloudUtilities::offsetCylinderCoefficients(primitive_process.request.inputs[i].expected_coefficients, map_offset);
	        }
	        clipping_pose.position.x -= map_offset[0];
	        clipping_pose.position.y -= map_offset[1];
	        clipping_pose.position.z -= map_offset[2];
	        //ROS_ERROR_STREAM("assigning clipping_pose to parameters");
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(clipping_pose.position.x);
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(clipping_pose.position.y);
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(clipping_pose.position.z);
	        // Convert quaternion message to RPY values: 
	        tf::Quaternion tf_quat;
	        tf::Matrix3x3 rpy_conversion_matrix;
	        tf::quaternionMsgToTF(clipping_pose.orientation, tf_quat);
	        rpy_conversion_matrix.setRotation(tf_quat);
	        double roll, pitch, yaw;
	        rpy_conversion_matrix.getRPY(roll, pitch, yaw);
	        roll -= map_offset[3];
	        pitch -= map_offset[4];
	        yaw -= map_offset[5];

	        tf_quat.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(tf_quat, clipping_pose.orientation);
	        // Assign CLIPPING BOX POSE
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(roll);
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(pitch);
	        primitive_process.request.inputs[i].tasks[0].parameters.push_back(yaw);

	        primitive_process.request.inputs[i].clipping_marker = PointcloudUtilities::makeClippingVisualization(clipping_pose, primitive_process.request.inputs[i].clipping_boundaries, primitive_process.request.inputs[i].tasks[0].name, i, marker_type);

	        primitive_process.request.inputs.push_back(primitive_process.request.inputs[i]);
	    }
	}

	std::vector<float> plane_offset_values(std::vector<float> expected_coefficients, std::vector<float> found_coefficients)
	{
		// Find RPY rotation to get aligned
		//   Project found and expected vectors onto XY plane E' = (E x Z) / |Z|,  F' = (F x Z) / |Z|
		//   Find angle between these two vectors acos((E' * F')/|E'|/|F'|)
		// Find XYZ translation to move onto plane 

	}

	std::vector<float> cylinder_offset_values(std::vector<float> expected_coefficients, std::vector<float> found_coefficients)
	{
		
	}

}
