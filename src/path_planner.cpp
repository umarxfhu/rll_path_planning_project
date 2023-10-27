// This file will mimic the behaviour of path_planner.py and client.py
// I will also modify the path_planner.launch to launch this cpp node instead
#include <vector>

#include "ros/ros.h"
#include "rll_planning_project/Move.h"
#include "rll_planning_project/CheckPath.h"
#include "rll_planning_project/GetStartGoal.h"
#include <rll_planning_project/planning_iface.h>
#include <rll_planning_project/planning_iface_simulation.h>

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
    // TODO: Fix get param syntax, not correct
    nh_ptr->getParam("/map_width", map_width);
    nh_ptr->getParam("/map_length", map_length);

    const std::string GET_START_GOAL_SRV_NAME = "get_start_goal";
    const std::string MOVE_SRV_NAME = "move";
    const std::string CHECK_PATH_SRV_NAME = "check_path";

    ros::ServiceClient get_start_goal_srv = nh_ptr->serviceClient<rll_planning_project::GetStartGoal>(GET_START_GOAL_SRV_NAME);
    ros::ServiceClient move_srv = nh_ptr->serviceClient<rll_planning_project::Move>(MOVE_SRV_NAME);
    ros::ServiceClient check_path_srv = nh_ptr->serviceClient<rll_planning_project::CheckPath>(CHECK_PATH_SRV_NAME, true);

    // Make a service call to get start and goal information using the PlanningIfaceBase class
    // PlanningIfaceBase planning_iface(*nh_ptr);

    rll_planning_project::GetStartGoal::Request start_goal_req;
    rll_planning_project::GetStartGoal::Response start_goal_resp;
    geometry_msgs::Pose2D pose_start;
    geometry_msgs::Pose2D pose_goal;

    if (get_start_goal_srv.call(start_goal_req, start_goal_resp))
    {
        pose_start = start_goal_resp.start;
        pose_goal = start_goal_resp.goal;
    }
    else
    {
        ROS_ERROR("[path_planner] Failed to call service get_start_goal_srv");
        return 1;
    }

    ROS_INFO("[path_planner][INFO] map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length);
    ROS_INFO("[path_planner][INFO] start pose: x %f, y %f, theta %f", pose_start.x, pose_start.y, pose_start.theta);
    ROS_INFO("[path_planner][INFO] goal pose: x %f, y %f, theta %f", pose_goal.x, pose_goal.y, pose_goal.theta);

    /*########################################
    # Implement path planning algorithm here #
    ########################################*/

    std::vector<geometry_msgs::Pose2D> path = {};

    // example motions for the gripper
    std::vector<std::vector<float>> motions = {
        // movement by 0.1m in positive or negative x-direction
        {0.1, 0, 0},
        {-0.1, 0, 0},
        // movement by 0.1m in positive or negative y-direction
        {0, 0.1, 0},
        {0, -0.1, 0},
        // rotation on the spot by 90°, clockwise or counterclockwise
        {0, 0, 1.57},
        {0, 0, 1.57},
        // rotation by 90° and movement into y or x direction (grinding curves)
        {0, 0.1, 1.57},
        {0.1, 0, -1.57}};

    /*###############################################
    # Example on how to use check_path functionality
    ###############################################*/

    geometry_msgs::Pose2D prev_pose = pose_start;

    for (auto motion : motions)
    {
        rll_planning_project::CheckPath::Request check_path_req;
        rll_planning_project::CheckPath::Response check_path_resp;
        geometry_msgs::Pose2D new_pose;

        new_pose.x = pose_start.x + motion[0];
        new_pose.y = pose_start.y + motion[1];
        new_pose.theta = pose_start.theta + motion[2];

        check_path_req.pose_start = prev_pose;
        check_path_req.pose_goal = new_pose;
        if (check_path_srv.call(check_path_req, check_path_resp))
        {
            if (check_path_resp.success)
            {
                ROS_INFO("[path_planner][INFO] CheckPath Success, adding position to path");
                path.push_back(new_pose);
            }
        }
        else
        {
            ROS_ERROR("[path_planner] Failed to call service check_path_srv");
            return 1;
        }
    }

    if (path.size() > 0)
    {
        ROS_INFO("[path_planner][INFO] A path was found, now trying to execute it");
        for (auto pose : path)
        {
            rll_planning_project::Move::Request move_req;
            rll_planning_project::Move::Response move_resp;
            move_req.pose = pose;
            if (move_srv.call(move_req, move_resp))
            {
            }
            else
            {
                ROS_ERROR("[path_planner] Failed to call service move_srv");
                return 1;
            }
        }
        return true;
    }
    ROS_INFO("[path_planner][INFO] No path to goal found");
    return false;
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