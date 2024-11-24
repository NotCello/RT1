#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>

const float MAX_LINEAR = 3.0;
const float MAX_ANGULAR = 5.0;

std::string selectTurtle() {
    std::string turtle;
    do {
        std::cout << "Enter turtle to control (turtle1 or turtle2): ";
        std::cin >> turtle;
    } while (turtle != "turtle1" && turtle != "turtle2");
    return turtle;
}

void sendCommand(ros::Publisher& pub) {
    float linear, angular;

    std::cout << "Enter linear velocity (-3 to 3): ";
    std::cin >> linear;
    linear = std::clamp(linear, -MAX_LINEAR, MAX_LINEAR);

    std::cout << "Enter angular velocity (-5 to 5): ";
    std::cin >> angular;
    angular = std::clamp(angular, -MAX_ANGULAR, MAX_ANGULAR);

    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    pub.publish(cmd);

    ROS_INFO("Command sent: linear=%.2f, angular=%.2f", linear, angular);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Spawn della seconda tartaruga
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 3.0;
    spawn_srv.request.y = 5.0;
    spawn_srv.request.name = "turtle2";
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 at (3.0, 5.0)");
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
        return 1;
    }

    // Publisher per le tartarughe
    ros::Publisher pub_t1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_t2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    while (ros::ok()) {
        std::string selected_turtle = selectTurtle();
        ros::Publisher& pub = (selected_turtle == "turtle1") ? pub_t1 : pub_t2;
        sendCommand(pub);
        ros::Duration(1.0).sleep();
    }

    return 0;
}
