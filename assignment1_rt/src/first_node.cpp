// ui_node.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>

// Constants
const float MAX_LINEAR = 3.0;
const float MAX_ANGULAR = 5.0;

// Clamp function
template <typename T>
T clamp(T value, T min, T max) {
    return std::max(std::min(value, max), min);
}

// Select turtle
std::string selectTurtle() {
    std::string turtle;
    do {
        std::cout << "Enter turtle to control (turtle1 or turtle2): ";
        std::cin >> turtle;
    } while (turtle != "turtle1" && turtle != "turtle2");
    return turtle;
}

// Send command and update velocity
void sendCommand(ros::Publisher& pub, ros::Publisher& vel_pub) {
    float linear_x, linear_y, angular_z;

    std::cout << "Enter linear velocity along x (-3 to 3): ";
    std::cin >> linear_x;
    linear_x = clamp(linear_x, -MAX_LINEAR, MAX_LINEAR);

    std::cout << "Enter linear velocity along y (-3 to 3): ";
    std::cin >> linear_y;
    linear_y = clamp(linear_y, -MAX_LINEAR, MAX_LINEAR);

    std::cout << "Enter angular velocity (-5 to 5): ";
    std::cin >> angular_z;
    angular_z = clamp(angular_z, -MAX_ANGULAR, MAX_ANGULAR);

    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular_z;

    pub.publish(cmd);

    // Publish velocity in km/h
    geometry_msgs::Twist vel_kmh;
    vel_kmh.linear.x = linear_x * 3.6;
    vel_kmh.linear.y = linear_y * 3.6;
    vel_kmh.angular.z = angular_z * 3.6; // Assuming angular velocity also in m/s for now.  Adjust if needed.
    vel_pub.publish(vel_kmh);

    ROS_INFO("Command sent: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f", linear_x, linear_y, angular_z);
}

int goals_reached = 0;
int goals_cancelled = 0;

bool goalCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data) {
        goals_reached++;
        res.message = "Goal reached!";
    } else {
        goals_cancelled++;
        res.message = "Goal cancelled!";
    }
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Spawn turtle2
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

    // Publishers
    ros::Publisher pub_t1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_t2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    ros::Publisher vel_pub_t1 = nh.advertise<geometry_msgs::Twist>("/turtle1/vel_kmh", 10);  // Velocity publisher
    ros::Publisher vel_pub_t2 = nh.advertise<geometry_msgs::Twist>("/turtle2/vel_kmh", 10);  // Velocity publisher


    // Service
    ros::ServiceServer service = nh.advertiseService("goal_service", goalCallback);

    while (ros::ok()) {
        std::string selected_turtle = selectTurtle();
        ros::Publisher& pub = (selected_turtle == "turtle1") ? pub_t1 : pub_t2;
        ros::Publisher& vel_pub = (selected_turtle == "turtle1") ? vel_pub_t1 : vel_pub_t2;

        sendCommand(pub, vel_pub);

        ros::Duration(1.0).sleep();
        ros::spinOnce(); // Important for service callbacks
    }

    return 0;
}
