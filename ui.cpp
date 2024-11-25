#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>

// Costanti
const float MAX_LINEAR = 3.0;  // Limite massimo velocità lineare
const float MAX_ANGULAR = 5.0; // Limite massimo velocità angolare

// Funzione clamp personalizzata
template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Funzione per selezionare la tartaruga da controllare
std::string selectTurtle() {
    std::string turtle;
    do {
        std::cout << "Enter turtle to control (turtle1 or turtle2): ";
        std::cin >> turtle;
    } while (turtle != "turtle1" && turtle != "turtle2");
    return turtle;
}

// Funzione per inviare i comandi
void sendCommand(ros::Publisher& pub) {
    float linear_x, linear_y, angular_z;

    // Input della velocità lungo x
    std::cout << "Enter linear velocity along x (-3 to 3): ";
    std::cin >> linear_x;
    linear_x = clamp(linear_x, -MAX_LINEAR, MAX_LINEAR);

    // Input della velocità lungo y
    std::cout << "Enter linear velocity along y (-3 to 3): ";
    std::cin >> linear_y;
    linear_y = clamp(linear_y, -MAX_LINEAR, MAX_LINEAR);

    // Input della velocità angolare
    std::cout << "Enter angular velocity (-5 to 5): ";
    std::cin >> angular_z;
    angular_z = clamp(angular_z, -MAX_ANGULAR, MAX_ANGULAR);

    // Creazione del comando Twist
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular_z;

    // Pubblicazione del comando
    pub.publish(cmd);

    // Log del comando inviato
    ROS_INFO("Command sent: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f", linear_x, linear_y, angular_z);
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
        // Selezione della tartaruga
        std::string selected_turtle = selectTurtle();

        // Seleziona il publisher corrispondente
        ros::Publisher& pub = (selected_turtle == "turtle1") ? pub_t1 : pub_t2;

        // Invio dei comandi
        sendCommand(pub);

        // Attendi un secondo prima del prossimo comando
        ros::Duration(1.0).sleep();
    }

    return 0;
}
