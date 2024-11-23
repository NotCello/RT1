#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>

void spawnTurtle(ros::NodeHandle &nh) {
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 5.0;
    spawnSrv.request.y = 5.0;
    spawnSrv.request.theta = 0.0;
    spawnSrv.request.name = "turtle2";

    if (spawnClient.call(spawnSrv)) {
        ROS_INFO("Turtle2 spawned successfully.");
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    ros::Publisher turtle1Pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher turtle2Pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    spawnTurtle(nh);

    while (ros::ok()) {
        int turtleChoice;
        double linear, angular;

        std::cout << "Select turtle to control (1 for turtle1, 2 for turtle2): ";
        std::cin >> turtleChoice;

        std::cout << "Enter linear velocity: ";
        std::cin >> linear;

        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        geometry_msgs::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;

        ros::Publisher selectedPub = (turtleChoice == 1) ? turtle1Pub : turtle2Pub;

        // Publish the command for 1 second
        ros::Rate rate(10);
        for (int i = 0; i < 10; i++) {
            selectedPub.publish(cmd);
            rate.sleep();
        }

        // Stop the turtle
        geometry_msgs::Twist stopCmd;
        selectedPub.publish(stopCmd);
    }

    return 0;
}
