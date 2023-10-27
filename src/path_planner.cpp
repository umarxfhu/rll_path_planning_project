// This file will mimic the behaviour of path_planner.py and client.py
// I will also modify the path_planner.launch to launch this cpp node instead
#include "ros/ros.h"
#include "rll_planning_project/Move.h"
#include "rll_planning_project/CheckPath.h"
#include "rll_planning_project/GetStartGoal.h"
#include <rll_planning_project/planning_iface.h>

#include <rll_move_client/util.h>
#include <rll_move_client/move_client_listener.h>
#include <rll_move_client/move_client_default.h>
#include <rll_move_client/move_client.h>

#include <geometry_msgs/Pose2D.h>

// Declare a global pointer to the NodeHandle
ros::NodeHandle *nh_ptr;

bool plan_to_goal(RLLDefaultMoveClient *const move_client)
{
    // Plan a path from Start to Goal

    ROS_INFO("[path_planner][INFO] Got a planning request");

    float map_width, map_length;
    nh_ptr->getParam("map_width", map_width);
    nh_ptr->getParam("map_length", map_length);

    const std::string GET_START_GOAL_SRV_NAME = "get_start_goal";
    const std::string MOVE_SRV_NAME = "move";
    const std::string CHECK_PATH_SRV_NAME = "check_path";

    ros::ServiceClient get_start_goal_srv = nh_ptr->serviceClient<rll_planning_project::GetStartGoal>(GET_START_GOAL_SRV_NAME);
    ros::ServiceClient move_srv = nh_ptr->serviceClient<rll_planning_project::Move>(MOVE_SRV_NAME);
    ros::ServiceClient check_path_srv = nh_ptr->serviceClient<rll_planning_project::CheckPath>(CHECK_PATH_SRV_NAME, true);

    // Make a service call to get start and goal information using the PlanningIfaceBase class
    PlanningIfaceBase planning_iface(*nh_ptr);

    rll_planning_project::GetStartGoal::Request start_goal_req;
    rll_planning_project::GetStartGoal::Response start_goal_resp;
    geometry_msgs::Pose2D pose_start;
    geometry_msgs::Pose2D pose_goal;

    if (planning_iface.getStartGoalSrv(start_goal_req, start_goal_resp))
    {
        pose_start = start_goal_resp.start;
        pose_goal = start_goal_resp.goal;
    }
    else
    {
        ROS_ERROR("[path_planner][ERROR] Failed to call getStartGoalSrv");
        // Handle the error
    }

    ROS_INFO("[path_planner][INFO] map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length);
    ROS_INFO("[path_planner][INFO] start pose: x %f, y %f, theta %f", pose_start.x, pose_start.y, pose_start.theta);
    ROS_INFO("[path_planner][INFO] goal pose: x %f, y %f, theta %f", pose_goal.x, pose_goal.y, pose_goal.theta);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    // Initialize the global pointer with the address of the local NodeHandle
    nh_ptr = &nh;

    RLLCallbackMoveClient<RLLDefaultMoveClient> client(&plan_to_goal);

    client.spin();
    return 0;
}