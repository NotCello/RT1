#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <cmath>

// Variabili globali
turtlesim::Pose turtle1Pose, turtle2Pose;
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

float calculateDistance() {
    return std::sqrt(std::pow(turtle1Pose.x - turtle2Pose.x, 2) +
                     std::pow(turtle1Pose.y - turtle2Pose.y, 2));
}

void stop_turtle(ros::Publisher& turtle_pub, geometry_msgs::Twist turtle_vel) {
    turtle_vel.linear.x = 0;
    turtle_vel.linear.y = 0;
    turtle_vel.angular.z = 0;
    turtle_pub.publish(turtle_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber turtle1Sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2Sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    ros::Subscriber activeTurtleSub = nh.subscribe("/active_turtle", 10, activeTurtleCallback);

    ros::Publisher distancePub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    ros::Publisher turtle1Pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher turtle2Pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    const float distanceThreshold = 1.0;
    ros::Rate rate(10);

    while (ros::ok()) {
        if (turtle1Received && turtle2Received && !activeTurtle.empty()) {
            float distance = calculateDistance();

            // Pubblica la distanza
            std_msgs::Float32 distanceMsg;
            distanceMsg.data = distance;
            distancePub.publish(distanceMsg);

            // Ferma la tartaruga attiva se troppo vicina
            if (distance < distanceThreshold) {
                ROS_WARN("Stopping %s: too close to the other turtle.", activeTurtle.c_str());

                geometry_msgs::Twist stopCmd;

                if (activeTurtle == "turtle1") {
                    stop_turtle(turtle1_pub, turtle1_vel);
                    
                } else if (activeTurtle == "turtle2") {
                    stop_turtle(turtle2_pub, turtle2_vel);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
