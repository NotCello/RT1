#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <cmath>
#include <vector>

const float SAFE_DISTANCE = 2.0;  // Safe distance to avoid collisions
const float SAFE_BOUNDARY = 1.5;  // Safety margin from walls
const float MIN_BOUND = 1.0;      // Minimum grid boundary
const float MAX_BOUND = 10.0;     // Maximum grid boundary

struct TurtleState {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher pub;
    std::string name;
};

std::vector<TurtleState> turtles(2);

// Callback to update turtle positions
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

// Compute distance between two turtles
float calculateDistance(const TurtleState& t1, const TurtleState& t2) {
    return std::sqrt(std::pow(t1.x - t2.x, 2) + std::pow(t1.y - t2.y, 2));
}

// Teleport a turtle to a safe position
void teleportTurtle(ros::NodeHandle& nh, TurtleState& turtle, const TurtleState& other) {
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle.name + "/teleport_absolute");

    turtlesim::TeleportAbsolute srv;
    srv.request.theta = 0.0;

    do {
        // Generate a random position within the grid but at a safe distance
        srv.request.x = MIN_BOUND + SAFE_BOUNDARY + (rand() % static_cast<int>(MAX_BOUND - 2 * SAFE_BOUNDARY));
        srv.request.y = MIN_BOUND + SAFE_BOUNDARY + (rand() % static_cast<int>(MAX_BOUND - 2 * SAFE_BOUNDARY));
    } while (calculateDistance(turtle, other) < SAFE_DISTANCE); // Ensure new position is safe

    if (teleport_client.call(srv)) {
        ROS_INFO("%s teleported to (%.2f, %.2f)", turtle.name.c_str(), srv.request.x, srv.request.y);
    } else {
        ROS_ERROR("Failed to teleport %s", turtle.name.c_str());
    }
}

// Handle turtle collisions by teleporting one turtle
void handleCollision(ros::NodeHandle& nh, TurtleState& t1, TurtleState& t2) {
    if (calculateDistance(t1, t2) < SAFE_DISTANCE) {
        ROS_WARN("Collision detected! Teleporting one turtle...");
        teleportTurtle(nh, t1, t2);
    }
}

// Check if a turtle is near a wall and teleport if necessary
void avoidWalls(ros::NodeHandle& nh, TurtleState& turtle, const TurtleState& other) {
    if (turtle.x < SAFE_BOUNDARY || turtle.x > (MAX_BOUND - SAFE_BOUNDARY) ||
        turtle.y < SAFE_BOUNDARY || turtle.y > (MAX_BOUND - SAFE_BOUNDARY)) {
        ROS_WARN("%s is near a wall! Teleporting...", turtle.name.c_str());
        teleportTurtle(nh, turtle, other);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Subscribers to track positions
    ros::Subscriber sub_t1_pose = nh.subscribe("/turtle1/pose", 10, poseCallbackTurtle1);
    ros::Subscriber sub_t2_pose = nh.subscribe("/turtle2/pose", 10, poseCallbackTurtle2);

    // Initialize turtle states
    turtles[0].name = "turtle1";
    turtles[1].name = "turtle2";

    ros::Rate rate(10);

    while (ros::ok()) {
        handleCollision(nh, turtles[0], turtles[1]);
        avoidWalls(nh, turtles[0], turtles[1]);
        avoidWalls(nh, turtles[1], turtles[0]);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
