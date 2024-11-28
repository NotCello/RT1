Turtlesim Collision and Wall Avoidance Project

Overview: 
This project demonstrates the interaction of two turtles in the turtlesim simulation using ROS. The turtles are programmed to avoid walls and each other, ensuring that:

They stop immediately upon collision and separate.
They stay safely away from the boundaries of the simulation environment.
They actively find positions where they are far from walls and far from each other.


Key Features
Collision Detection and Handling:
When two turtles collide (distance < 1.2 units), the turtle in motion stops immediately.
After stopping, both turtles move in opposite directions to increase the distance between them.


Wall Avoidance:
When a turtle approaches the boundary of the simulation grid, it makes small corrective movements along 
x or y, followed by a slight rotation to reorient itself.
This ensures the turtles stay within the simulation bounds.

User Interaction:
The user can control either of the two turtles by setting linear velocities along 
𝑥 and y and an angular velocity for rotation.




Code Description
1. Node: ui_node.cpp
The ui_node provides a user interface for controlling the movement of the turtles.

Key Features:

User-Selected Turtle Control:
The user selects which turtle (turtle1 or turtle2) to control.
Three Independent Velocities:
Linear velocity along the 
𝑥 (x-axis).

Linear velocity along the 
𝑦 (y-axis).

Angular velocity (rotation).


Safety Bounds:
The entered velocities are clamped to ensure they remain within predefined limits:
𝑥 and y: [-3, 3]
Angular velocity: [-5, 5]

Workflow:
The user is prompted to select a turtle.
The user enters velocity values for 𝑥, 𝑦, and rotation.
The node publishes the commands to the selected turtle's velocity topic (/turtleX/cmd_vel).

2. Node: distance_node.cpp
The distance_node ensures the turtles avoid collisions and walls through automatic adjustments.

Key Features:

Collision Handling:
Stops the moving turtle immediately upon detecting a collision.
Moves both turtles in opposite directions and logs the new distance after separation.

Wall Avoidance:
Detects if a turtle is too close to the boundary.
Executes small corrective movements along 
𝑥 or 𝑦 and rotates slightly to keep the turtle safely within bounds.
Workflow:

Continuously monitors the positions of both turtles via /turtleX/pose.
If a collision is detected:
Stops the moving turtle.
Moves both turtles away from each other.
If a turtle approaches the boundary:
Executes minimal adjustments to reposition it within the safe zone.
Simulation Logic

Collision Handling:

The node calculates the squared distance between turtles.
If the distance is below the safe threshold (1.2 units):
The moving turtle stops immediately.
Both turtles are moved in opposite directions to increase the separation.

Wall Avoidance:

If a turtle's position is within the safety margin (1.5 units from the boundary):
It moves slightly along 𝑥 or 𝑦 to exit the critical area.
A slight rotation ensures the turtle reorients itself for continued movement.




How to Run the Simulation
Launch the Turtlesim Simulation:

rosrun turtlesim turtlesim_node
This initializes the simulation environment.

Start the ui_node:

rosrun assignment_rt1 ui_node
Enables user control of the turtles.

Start the distance_nod
rosrun assignment_rt1 distance_node
Activates collision handling and wall avoidance mechanisms.

Expected Behavior
Collision Handling:
When turtles collide, they stop immediately and move apart.
The new distance between turtles is logged in the terminal.
Wall Avoidance:
Turtles approaching the boundary make small corrective movements and rotate to reorient themselves.
User Control:
The user can control a turtle's linear and angular velocities through the terminal.

Code Structure
ui_node.cpp:
Inputs:
User-selected turtle (turtle1 or turtle2).
Linear velocities (𝑥 and 𝑦).
Angular velocity (rotation).
Outputs:
Publishes velocity commands to /turtleX/cmd_vel.
distance_node.cpp:
Inputs:
Subscribes to /turtle1/pose and /turtle2/pose to get current positions.
Outputs:
Publishes corrective velocity commands to /turtleX/cmd_vel when:
Turtles are too close.
Turtles approach the boundary.

Dynamic Collision Handling:
Introduce a smoother separation mechanism for collisions.
Leader-Follower Behavior:
Implement a mode where one turtle follows the other while maintaining a safe distance.
Obstacle Avoidance:
Extend the simulation to include additional virtual obstacles.

Conclusion
This project highlights basic ROS concepts like publishers, subscribers, and services in an interactive simulation. The turtles are programmed with collision avoidance and wall avoidance behaviors, ensuring they operate safely within the simulation. This combination of user control and automated safety features creates a robust and engaging simulation.
