#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

const float SAFE_DISTANCE_SQ = 1.44;  // Distanza² per evitare collisioni (1.2²)
const float MIN_BOUND = 1.0;
const float MAX_BOUND = 10.0;

struct TurtleState {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher pub;
    std::string name;
};

std::vector<TurtleState> turtles(2);

void poseCallbackTurtle1(const turtlesim::Pose::ConstPtr& msg) {
    turtles[0].x = msg->x;
    turtles[0].y = msg->y;
    turtles[0].theta = msg->theta;
}

void poseCallbackTurtle2(const turtlesim::Pose::ConstPtr& msg) {
    turtles[1].x = msg->x;
    turtles[1].y = msg->y;
    turtles[1].theta = msg->theta;
}

void stopTurtle(TurtleState& turtle) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    turtle.pub.publish(stop_cmd);
}

void moveTurtleBackwards(TurtleState& turtle) {
    geometry_msgs::Twist reverse_cmd;
    reverse_cmd.linear.x = -0.5;
    turtle.pub.publish(reverse_cmd);
    ros::Duration(0.2).sleep();
    stopTurtle(turtle);
}

float calculateSquaredDistance(const TurtleState& t1, const TurtleState& t2) {
    return std::pow(t1.x - t2.x, 2) + std::pow(t1.y - t2.y, 2);
}

void checkBoundaries(TurtleState& turtle) {
    if (turtle.x < MIN_BOUND || turtle.x > MAX_BOUND || turtle.y < MIN_BOUND || turtle.y > MAX_BOUND) {
        ROS_WARN("%s is near boundary! Moving back...", turtle.name.c_str());
        moveTurtleBackwards(turtle);
    }
}

void timerCallback(const ros::TimerEvent&) {
    // Calcolo della distanza²
    float distance_sq = calculateSquaredDistance(turtles[0], turtles[1]);
    std_msgs::Float32 distance_msg;
    distance_msg.data = std::sqrt(distance_sq);
    ros::NodeHandle nh;
    auto pub_distance = nh.advertise<std_msgs::Float32>("/turtles/rel_distance", 10);
    pub_distance.publish(distance_msg);

    if (distance_sq < SAFE_DISTANCE_SQ) {
        ROS_WARN("Turtles too close! Stopping turtle2...");
        stopTurtle(turtles[1]);
        moveTurtleBackwards(turtles[1]);
    }

    // Controlla i limiti per entrambe le tartarughe
    checkBoundaries(turtles[0]);
    checkBoundaries(turtles[1]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Imposta i subscriber per le posizioni
    ros::Subscriber sub_t1_pose = nh.subscribe("/turtle1/pose", 10, poseCallbackTurtle1);
    ros::Subscriber sub_t2_pose = nh.subscribe("/turtle2/pose", 10, poseCallbackTurtle2);

    // Inizializza gli stati delle tartarughe
    turtles[0].name = "turtle1";
    turtles[0].pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtles[1].name = "turtle2";
    turtles[1].pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Timer per il controllo periodico
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    ros::spin();
    return 0;
}
