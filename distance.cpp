#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

const float SAFE_DISTANCE = 1.2;  // Soglia per evitare collisioni
const float MIN_BOUND = 1.0;     // Limite minimo della griglia
const float MAX_BOUND = 10.0;    // Limite massimo della griglia

double turtle_x[2], turtle_y[2], turtle_theta[2]; // Posizioni delle tartarughe
bool turtle_moving[2] = {false, false};           // Stato di movimento delle tartarughe

void poseCallbackTurtle1(const turtlesim::Pose::ConstPtr& msg) {
    turtle_x[0] = msg->x;
    turtle_y[0] = msg->y;
    turtle_theta[0] = msg->theta;
}

void poseCallbackTurtle2(const turtlesim::Pose::ConstPtr& msg) {
    turtle_x[1] = msg->x;
    turtle_y[1] = msg->y;
    turtle_theta[1] = msg->theta;
}

float calculateDistance() {
    return std::sqrt(std::pow(turtle_x[0] - turtle_x[1], 2) + std::pow(turtle_y[0] - turtle_y[1], 2));
}

void stopTurtle(ros::Publisher& pub) {
    geometry_msgs::Twist stop_vel;
    stop_vel.linear.x = 0.0;
    stop_vel.angular.z = 0.0;
    pub.publish(stop_vel);
}

void reverseTurtle(int id, ros::Publisher& pub) {
    geometry_msgs::Twist reverse_vel;
    reverse_vel.linear.x = -0.5; // Velocità opposta per tornare indietro
    pub.publish(reverse_vel);
    ros::Duration(0.1).sleep(); // Breve pausa
    stopTurtle(pub);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_pose_t1 = nh.subscribe("/turtle1/pose", 10, poseCallbackTurtle1);
    ros::Subscriber sub_pose_t2 = nh.subscribe("/turtle2/pose", 10, poseCallbackTurtle2);

    ros::Publisher pub_t1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_t2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    ros::Publisher pub_distance = nh.advertise<std_msgs::Float32>("/turtles/rel_distance", 10);

    ros::Rate rate(10);
    while (ros::ok()) {
        float distance = calculateDistance();

        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        pub_distance.publish(distance_msg);

        if (distance < SAFE_DISTANCE) {
            ROS_WARN("Turtles too close! Distance: %.2f", distance);
            reverseTurtle(0, pub_t1);
        }

        // Controllo dei limiti della griglia
        for (int i = 0; i < 2; ++i) {
            if (turtle_x[i] < MIN_BOUND || turtle_x[i] > MAX_BOUND ||
                turtle_y[i] < MIN_BOUND || turtle_y[i] > MAX_BOUND) {
                ROS_WARN("Turtle %d near boundary! Stopping...", i + 1);
                reverseTurtle(i, (i == 0 ? pub_t1 : pub_t2));
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
