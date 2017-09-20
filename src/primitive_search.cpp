
#include "pointcloud_primitive_search/primitive_search.h"



PrimitiveSearch::PrimitiveSearch()
{
	client_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
    server_ = nh_.advertiseService("primitive_search", &PrimitiveSearch::primitiveSearch, this);
    //ros::spin();
}

PrimitiveSearch::PrimitiveSearch(std::vector< std::vector<float> > expected_coefficients, std::string yaml_file_name)
{
	client_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
    server_ = nh_.advertiseService("primitive_search", &PrimitiveSearch::primitiveSearch, this);

}

bool PrimitiveSearch::primitiveSearch(pointcloud_primitive_search::primitive_process::Request &req, pointcloud_primitive_search::primitive_process::Response &res)
{
// -----------------------------------------------------------------------------------------
// -------------------------- Actual loop to look for primitives! --------------------------
    bool match_found = false;
    bool process_failed = false;
    for(int i=0; i<rich_processes_.size(); i++)
    {
        ROS_DEBUG_STREAM("[PrimitiveSearch] Starting a new primitive search...");
        ros::Time time_start = ros::Time::now();

        pointcloud_processing_server::pointcloud_process current_process;
        current_process.request.tasks.push_back(req.inputs[i].tasks[0]);
        current_process.request.tasks.push_back(req.inputs[i].tasks[1]);
        current_process.request.pointcloud = req.pointcloud;
        current_process.request.min_cloud_size = req.inputs[i].min_cloud_size;

        while ( !match_found && !process_failed )
        {
            int failure_count = 0;
            while ( !client_.call(current_process) ) // If we couldn't read output, the service probably isn't up yet --> retry
            {
                ROS_ERROR_STREAM("[PrimitiveSearch] Service 'pointcloud_service' returned false during process " << current_process.request.tasks[1].name << " - may not be up yet. Trying again in 2 seconds...");
                if ( failure_count > 5 )
                    break;
                ros::Duration(1).sleep();
            }

            process_failed = current_process.response.failed;

            if(process_failed)
            {
                ROS_ERROR_STREAM("[PrimitiveSearch] Segmentation process failed during 'searchForPrimitives' call, on process " << current_process.request.tasks[1].name << "!");
                for (int j=i+1; j<rich_processes_.size(); j++)
                    current_process.response.failed = true; 
                break;
            }

            switch(current_process.request.tasks[1].type_ind)
            {
                case 5: // IS A PLANE
                    if( PointcloudUtilities::checkPlaneValidity(req.inputs[i].expected_coefficients, current_process.response.task_results[1].primitive_coefficients, req.inputs[i].check_orientations, req.inputs[i].check_distances, req.inputs[i].angle_threshold, req.inputs[i].offset_threshold) )
                    {
                        match_found = true;
                        ROS_DEBUG_STREAM("[PrimitiveSearch] Found an acceptable plane! Matched to task " << current_process.request.tasks[1].name << ".");
                    }
                    break;

                case 6: // IS A CYLINDER
                    if( PointcloudUtilities::checkCylinderValidity(req.inputs[i].expected_coefficients, current_process.response.task_results[1].primitive_coefficients, req.inputs[i].check_radii, req.inputs[i].check_orientations, req.inputs[i].check_distances, req.inputs[i].radius_threshold, req.inputs[i].angle_threshold, req.inputs[i].offset_threshold) )
                    {
                        match_found = true;
                        ROS_DEBUG_STREAM("[PrimitiveSearch] Found an acceptable cylinder! Matched to task " << current_process.request.tasks[1].name << ".");
                    }
                    break;
            }

            if(!match_found)
            {
                ROS_DEBUG_STREAM("[PrimitiveSearch] Found primitive is not acceptable. Removing it and trying again...");
                current_process.request.pointcloud = current_process.response.task_results[1].remaining_pointcloud;
            }
            // realistically, remove_cloud should probably always be TRUE for this kind of work... but for flexibility's sake: 
            else if(current_process.request.tasks[1].remove_cloud) 
                ROS_DEBUG_STREAM("[PrimitiveSearch] Found primitive for task " << current_process.request.tasks[1].name << " is acceptable! Continuing on...");
        }  
        match_found = false;        // might need to do something more deliberate to store found primitives? 
        process_failed = false;

        // Populate service response: 
        pointcloud_primitive_search::search_output search_output;
        search_output.task_results = current_process.response.task_results;
        search_output.failed = current_process.response.failed;
        search_output.failed_during_task_num = current_process.response.failed_during_task_num;
        res.outputs.push_back(search_output);

        //req.process_pub.publish(rich_processes_[i].process);
        //req.marker_pub.publish(rich_processes_[i].clipping_marker);

        ros::Duration search_duration = ros::Time::now() - time_start;
        ROS_INFO_STREAM("[PrimitiveSearch] Finished a primitive search! Entire search took " << search_duration << " seconds.");
    }

    return true;
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "test_segmentation");

//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//        ros::console::notifyLoggerLevelsChanged();

    PrimitiveSearch primitive_search;

    ros::spin();

    ros::Duration(2.0).sleep();

	return 0;
}