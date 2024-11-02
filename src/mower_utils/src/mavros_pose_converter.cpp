//
// Created by Clemens Elflein on 22.02.22.
// Adapted for MAVROS by [Pepeuch] on [11/1/2024]
//
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped out;
std::string frame;

void pose_received(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Copier les données de pose et adapter le format pour le message de covariance
    out.header = msg->header;
    out.pose.pose = msg->pose;
    out.pose.pose.position.z = 0;  // Mettre Z à zéro si nécessaire
    out.header.frame_id = frame;
    pose_pub.publish(out);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_pose_converter");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    // Obtenir le topic source (par défaut pour MAVROS)
    std::string topic;
    paramNh.param("topic", topic, std::string("/mavros/local_position/pose"));
    paramNh.param("frame", frame, std::string("map"));

    // Définir le topic cible pour la pose convertie
    std::string target_topic = topic + "/converted";

    ROS_INFO_STREAM("Converting " << topic << " to " << target_topic);

    pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovarianceStamped>(target_topic, 10, false);

    // Abonnement au topic MAVROS pour la position locale
    ros::Subscriber s = n.subscribe(topic, 10, pose_received);

    ros::spin();

    return 0;
}
