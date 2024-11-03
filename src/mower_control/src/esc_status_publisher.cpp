#include "ros/ros.h"
#include "mower_msgs/MowerControlSrv.h"

// Service pour lancer la tonte
bool handleMowerControl(mower_msgs::MowerControlSrv::Request &req,
                        mower_msgs::MowerControlSrv::Response &res) {
    if (req.start_mowing) {
        ROS_INFO("Commande reçue : Démarrage de la tonte");
        res.success = true;
        res.message = "Tonte démarrée avec succès.";
    } else {
        ROS_INFO("Commande reçue : Arrêt de la tonte");
        res.success = true;
        res.message = "Tonte arrêtée avec succès.";
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_control_service");
    ros::NodeHandle nh;

    // Créer le service
    ros::ServiceServer service = nh.advertiseService("mower_control_service", handleMowerControl);
    ROS_INFO("Service de contrôle de la tondeuse prêt à recevoir des commandes.");

    ros::spin();
    return 0;
}
