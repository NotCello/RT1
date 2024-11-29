#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

const float SAFE_DISTANCE_SQ = 1.44;  // distance to avoid collisions between turtles
const float SAFE_BOUNDARY = 1.5;      // safety margin from walls
const float MIN_BOUND = 1.0;          // minimum limit of the grid 
const float MAX_BOUND = 10.0;         // maximum limit of the grid 

struct TurtleState {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher pub;
    std::string name;
    bool is_moving = false; // indicate if the turtle is moving
};

std::vector<TurtleState> turtles(2);

// callback to update the position of turtle1
void poseCallbackTurtle1(const turtlesim::Pose::ConstPtr& msg) {
    turtles[0].x = msg->x;
    turtles[0].y = msg->y;
    turtles[0].theta = msg->theta;
}

// callback to update the position of turtle2
void poseCallbackTurtle2(const turtlesim::Pose::ConstPtr& msg) {
    turtles[1].x = msg->x;
    turtles[1].y = msg->y;
    turtles[1].theta = msg->theta;
}

// stop the turtle
void stopTurtle(TurtleState& turtle) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    turtle.pub.publish(stop_cmd);
    turtle.is_moving = false;
}

//movement to avoid the wall with little move
void avoidWalls(TurtleState& turtle, const TurtleState& other) {
    if (turtle.x < SAFE_BOUNDARY || turtle.x > (MAX_BOUND - SAFE_BOUNDARY) ||
        turtle.y < SAFE_BOUNDARY || turtle.y > (MAX_BOUND - SAFE_BOUNDARY)) {
        float distance = std::sqrt(std::pow(turtle.x - other.x, 2) + std::pow(turtle.y - other.y, 2));
        ROS_WARN("%s is near a wall! Distance to other turtle: %.2f. Adjusting position...", turtle.name.c_str(), distance);

        geometry_msgs::Twist adjust_cmd;

        // minimum moviment to exit from the grid
        if (turtle.x < SAFE_BOUNDARY) {
            adjust_cmd.linear.x = 1; 
        } else if (turtle.x > (MAX_BOUND - SAFE_BOUNDARY)) {
            adjust_cmd.linear.x = -1;
        }

        if (turtle.y < SAFE_BOUNDARY) {
            adjust_cmd.linear.y = 1; 
        } else if (turtle.y > (MAX_BOUND - SAFE_BOUNDARY)) {
            adjust_cmd.linear.y = -1;
        }

        //rotate a little to change the orientation
        adjust_cmd.angular.z = 0.4;

        //publish the comand
        turtle.pub.publish(adjust_cmd);
        ros::Duration(0.3).sleep();

        //stopthe turtle
        stopTurtle(turtle);
    }
}

// compute the distance between the two turtles
float calculateSquaredDistance(const TurtleState& t1, const TurtleState& t2) {
    return std::sqrt(std::pow(t1.x - t2.x, 2) + std::pow(t1.y - t2.y, 2));
}

// handles the collision between the turtles
void handleCollision(TurtleState& t1, TurtleState& t2) {
    float distance_sq = calculateSquaredDistance(t1, t2);
    if (distance_sq < SAFE_DISTANCE_SQ) {
        float distance = std::sqrt(distance_sq);
        ROS_WARN("Collision detected! Distance: %.2f. Stopping moving turtle...", distance);

        // stop instastaniously both the turtles
        if (t1.is_moving) {
            stopTurtle(t1);
        } else if (t2.is_moving) {
            stopTurtle(t2);
        }

        // after the collisioon I separate the turtles
        ROS_INFO("Separating turtles...");
        geometry_msgs::Twist separation_cmd;

        // I move t1 and t2 in the opposite directions
        separation_cmd.linear.x = 0.5; 
                separation_cmd.linear.y = 0.5;
        separation_cmd.angular.z = -1.0;
        t1.pub.publish(separation_cmd);

        separation_cmd.linear.x = -0.5; 
                separation_cmd.linear.y = -0.5;
        separation_cmd.angular.z = 1.0; 
        t2.pub.publish(separation_cmd);

        ros::Duration(0.5).sleep(); // wait that the turtle are far 
        stopTurtle(t1);
        stopTurtle(t2);

        // Stamp the new distance
        distance_sq = calculateSquaredDistance(t1, t2);
        distance = std::sqrt(distance_sq);
        ROS_INFO("New distance after adjustment: %.2f", distance);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // set the subscriber for the positions
    ros::Subscriber sub_t1_pose = nh.subscribe("/turtle1/pose", 10, poseCallbackTurtle1);
    ros::Subscriber sub_t2_pose = nh.subscribe("/turtle2/pose", 10, poseCallbackTurtle2);

    // initializr the state of the turtles
    turtles[0].name = "turtle1";
    turtles[0].pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtles[1].name = "turtle2";
    turtles[1].pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        // check the collisions and handle them
        handleCollision(turtles[0], turtles[1]);

        // check that every turtle is far from walls
        avoidWalls(turtles[0], turtles[1]);
        avoidWalls(turtles[1], turtles[0]);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
} 
