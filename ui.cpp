#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>

// constante
const float MAX_LINEAR = 3.0;  // maximum limit linear speed
const float MAX_ANGULAR = 5.0; // maximum limit angular speed

// clamp function
template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// select the turtle to control
std::string selectTurtle() {
    std::string turtle;
    do {
        std::cout << "Enter turtle to control (turtle1 or turtle2): ";
        std::cin >> turtle;
    } while (turtle != "turtle1" && turtle != "turtle2");
    return turtle;
}

// function to send commands
void sendCommand(ros::Publisher& pub) {
    float linear_x, linear_y, angular_z;

    //imput of the velocity along x
    std::cout << "Enter linear velocity along x (-3 to 3): ";
    std::cin >> linear_x;
    linear_x = clamp(linear_x, -MAX_LINEAR, MAX_LINEAR);

    //imput of the velocity along x
    std::cout << "Enter linear velocity along y (-3 to 3): ";
    std::cin >> linear_y;
    linear_y = clamp(linear_y, -MAX_LINEAR, MAX_LINEAR);

    //imput of theangular velocity
    std::cout << "Enter angular velocity (-5 to 5): ";
    std::cin >> angular_z;
    angular_z = clamp(angular_z, -MAX_ANGULAR, MAX_ANGULAR);

    // creation of the twist command
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular_z;

    // publish the command
    pub.publish(cmd);

    // log the command sent
    ROS_INFO("Command sent: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f", linear_x, linear_y, angular_z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // spawn the second turtle
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

    // turtles publisher
    ros::Publisher pub_t1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_t2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    while (ros::ok()) {
        // select the turtle
        std::string selected_turtle = selectTurtle();

        // select the corrispondence publisher
        ros::Publisher& pub = (selected_turtle == "turtle1") ? pub_t1 : pub_t2;

        // send commands
        sendCommand(pub);

        // wait a second before the next command
        ros::Duration(1.0).sleep();
    }

    return 0;
}
