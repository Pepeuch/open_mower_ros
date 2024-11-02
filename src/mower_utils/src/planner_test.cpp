//
// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// Adapted for MAVROS by [Pepeuch] on [11/1/2024]
//
#include "mower_map/GetMowingAreaSrv.h"
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include <geometry_msgs/PoseStamped.h>

ros::ServiceClient pathClient, mapClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner_test");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    int area_index = paramNh.param("area_index", 0);
    int outline_count = paramNh.param("outline_count", 4);

    // Publisher to MAVROS setpoint position topic
    ros::Publisher path_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100, true);

    pathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
    mapClient = n.serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");

    ROS_INFO("Waiting for map server");
    if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Map server service not found.");
        return 2;
    }

    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = area_index;

    if (!mapClient.call(mapSrv)) {
        ROS_ERROR_STREAM("Error loading mowing area");
        return 1;
    }

    slic3r_coverage_planner::PlanPath pathSrv;
    pathSrv.request.angle = 0;
    pathSrv.request.outline_count = outline_count;
    pathSrv.request.outline = mapSrv.response.area.area;
    pathSrv.request.holes = mapSrv.response.area.obstacles;
    pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
    pathSrv.request.distance = 0.13;
    pathSrv.request.outer_offset = 0.05;

    ros::Duration loop_time(1);
    while (ros::ok()) {
        if (!pathClient.call(pathSrv)) {
            ROS_ERROR_STREAM("Error getting path area");
        } else {
            ROS_INFO_STREAM("Got path");

            // Publish each point in the path to MAVROS
            for (const auto& pose : pathSrv.response.path.poses) {
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.header.frame_id = "map";  // Set frame of reference
                poseStamped.pose = pose.pose;

                path_pub.publish(poseStamped);
                ros::Duration(0.1).sleep();  // Small delay to simulate navigation between points
            }
        }
        loop_time.sleep();
    }

    return 0;
}
