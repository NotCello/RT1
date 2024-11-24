#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <cmath>
#include <string>

// Variabili globali per salvare i dati
turtlesim::Pose turtle1Pose, turtle2Pose;
geometry_msgs::Twist activeVelocity;
std::string activeTurtle = ""; // Nome della tartaruga attiva
bool turtle1Received = false, turtle2Received = false;

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1Pose = *msg;
    turtle1Received = true;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2Pose = *msg;
    turtle2Received = true;
}

void activeTurtleCallback(const std_msgs::String::ConstPtr &msg) {
    activeTurtle = msg->data;
}

void activeVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    activeVelocity = *msg;
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
    ros::Subscriber activeTurtleSub = nh.subscribe("/active_turtle", 10, activeTurtleCallback);
    ros::Subscriber activeVelocitySub = nh.subscribe("/active_velocity", 10, activeVelocityCallback);

    ros::Publisher distancePub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    ros::Publisher turtle1Pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher turtle2Pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    const float distanceThreshold = 1.0;
    ros::Rate rate(10);

    while (ros::ok()) {
        if (turtle1Received && turtle2Received && !activeTurtle.empty()) {
            // Calcola la distanza tra le tartarughe
            float distance = calculateDistance();

            // Pubblica la distanza
            std_msgs::Float32 distanceMsg;
            distanceMsg.data = distance;
            distancePub.publish(distanceMsg);

            // Stampa la distanza calcolata
            ROS_INFO("Distance between turtle1 and turtle2: %.2f", distance);

            // Ferma la tartaruga attiva se troppo vicina
            if (distance < distanceThreshold) {
                ROS_WARN("Stopping %s: too close to the other turtle.", activeTurtle.c_str());

                geometry_msgs::Twist stopCmd; // Velocità zero

                if (activeTurtle == "turtle1") {
                    turtle1Pub.publish(stopCmd);
                } else if (activeTurtle == "turtle2") {
                    turtle2Pub.publish(stopCmd);
                }
            } else {
                ROS_INFO("Active Turtle: %s, Linear Velocity: %.2f, Angular Velocity: %.2f",
                         activeTurtle.c_str(), activeVelocity.linear.x, activeVelocity.angular.z);
            }
        }

        rate.sleep();
    }

    return 0;
}
