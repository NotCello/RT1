#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>

// Variabili globali per salvare le posizioni
turtlesim::Pose turtle1Pose, turtle2Pose;
bool turtle1Received = false, turtle2Received = false;
bool stopTurtle2 = false; // Flag per fermare turtle2

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1Pose = *msg;
    turtle1Received = true;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2Pose = *msg;
    turtle2Received = true;
}

float calculateDistance() {
    return std::sqrt(std::pow(turtle1Pose.x - turtle2Pose.x, 2) +
                     std::pow(turtle1Pose.y - turtle2Pose.y, 2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber turtle1Sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2Sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    ros::Publisher distancePub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    ros::Publisher turtle2Pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    const float distanceThreshold = 1.0;
    ros::Rate rate(10);

    while (ros::ok()) {
        if (turtle1Received && turtle2Received) {
            float distance = calculateDistance();

            // Pubblica la distanza
            std_msgs::Float32 distanceMsg;
            distanceMsg.data = distance;
            distancePub.publish(distanceMsg);

            // Controlla se fermare turtle2
            if (distance < distanceThreshold || 
                turtle2Pose.x < 1.0 || turtle2Pose.x > 10.0 ||
                turtle2Pose.y < 1.0 || turtle2Pose.y > 10.0) {
                stopTurtle2 = true;
                ROS_WARN("Stopping turtle2: too close to turtle1 or boundaries.");
            } else {
                stopTurtle2 = false;
            }

            // Invia comando di arresto se necessario
            if (stopTurtle2) {
                geometry_msgs::Twist stopCmd;
                turtle2Pub.publish(stopCmd);
            }
        }

        rate.sleep();
    }

    return 0;
}
