#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

const float SAFE_DISTANCE_SQ = 1.44;  // Distanza² per evitare collisioni tra tartarughe (1.2²)
const float SAFE_BOUNDARY = 1.5;      // Margine di sicurezza dai muri
const float MIN_BOUND = 1.0;          // Limite minimo della griglia
const float MAX_BOUND = 10.0;         // Limite massimo della griglia

struct TurtleState {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher pub;
    std::string name;
    bool is_moving = false; // Indica se la tartaruga è in movimento
};

std::vector<TurtleState> turtles(2);

// Callback per aggiornare la posizione di turtle1
void poseCallbackTurtle1(const turtlesim::Pose::ConstPtr& msg) {
    turtles[0].x = msg->x;
    turtles[0].y = msg->y;
    turtles[0].theta = msg->theta;
}

// Callback per aggiornare la posizione di turtle2
void poseCallbackTurtle2(const turtlesim::Pose::ConstPtr& msg) {
    turtles[1].x = msg->x;
    turtles[1].y = msg->y;
    turtles[1].theta = msg->theta;
}

// Ferma la tartaruga
void stopTurtle(TurtleState& turtle) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    turtle.pub.publish(stop_cmd);
    turtle.is_moving = false;
}

// Movimento per evitare i muri con spostamenti minimi
void avoidWalls(TurtleState& turtle, const TurtleState& other) {
    if (turtle.x < SAFE_BOUNDARY || turtle.x > (MAX_BOUND - SAFE_BOUNDARY) ||
        turtle.y < SAFE_BOUNDARY || turtle.y > (MAX_BOUND - SAFE_BOUNDARY)) {
        float distance = std::sqrt(std::pow(turtle.x - other.x, 2) + std::pow(turtle.y - other.y, 2));
        ROS_WARN("%s is near a wall! Distance to other turtle: %.2f. Adjusting position...", turtle.name.c_str(), distance);

        geometry_msgs::Twist adjust_cmd;

        // Movimento minimo per uscire dalla zona critica
        if (turtle.x < SAFE_BOUNDARY) {
            adjust_cmd.linear.x = 0.2; // Avanza lungo x
        } else if (turtle.x > (MAX_BOUND - SAFE_BOUNDARY)) {
            adjust_cmd.linear.x = -0.2; // Indietro lungo x
        }

        if (turtle.y < SAFE_BOUNDARY) {
            adjust_cmd.linear.y = 0.2; // Avanza lungo y
        } else if (turtle.y > (MAX_BOUND - SAFE_BOUNDARY)) {
            adjust_cmd.linear.y = -0.2; // Indietro lungo y
        }

        // Ruota leggermente per cambiare orientamento
        adjust_cmd.angular.z = 0.5;

        // Pubblica il comando
        turtle.pub.publish(adjust_cmd);
        ros::Duration(0.3).sleep();

        // Ferma il movimento
        stopTurtle(turtle);
    }
}

// Calcola la distanza² tra due tartarughe
float calculateSquaredDistance(const TurtleState& t1, const TurtleState& t2) {
    return std::pow(t1.x - t2.x, 2) + std::pow(t1.y - t2.y, 2);
}

// Gestisce la collisione tra tartarughe
void handleCollision(TurtleState& t1, TurtleState& t2) {
    float distance_sq = calculateSquaredDistance(t1, t2);
    if (distance_sq < SAFE_DISTANCE_SQ) {
        float distance = std::sqrt(distance_sq);
        ROS_WARN("Collision detected! Distance: %.2f. Stopping moving turtle...", distance);

        // Ferma immediatamente entrambe le tartarughe
        if (t1.is_moving) {
            stopTurtle(t1);
        } else if (t2.is_moving) {
            stopTurtle(t2);
        }

        // Dopo la collisione, separa le tartarughe
        ROS_INFO("Separating turtles...");
        geometry_msgs::Twist separation_cmd;

        // Muovi t1 e t2 in direzioni opposte
        separation_cmd.linear.x = 0.5; // Avanti
        separation_cmd.angular.z = -1.0; // Rotazione opposta
        t1.pub.publish(separation_cmd);

        separation_cmd.linear.x = -0.5; // Indietro
        separation_cmd.angular.z = 1.0; // Rotazione
        t2.pub.publish(separation_cmd);

        ros::Duration(0.5).sleep(); // Aspetta che si allontanino
        stopTurtle(t1);
        stopTurtle(t2);

        // Stampa la nuova distanza
        distance_sq = calculateSquaredDistance(t1, t2);
        distance = std::sqrt(distance_sq);
        ROS_INFO("New distance after adjustment: %.2f", distance);
    }
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

    ros::Rate rate(10);

    while (ros::ok()) {
        // Controlla per collisioni e gestiscile
        handleCollision(turtles[0], turtles[1]);

        // Controlla che ogni tartaruga non si avvicini ai muri
        avoidWalls(turtles[0], turtles[1]);
        avoidWalls(turtles[1], turtles[0]);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
