#foolturtle package

Package to illustrate the ROS 2 publish/subscribe anonymous message passing.

Used in the **Communications using topics** tutorial: https://sir.upc.edu/projects/ros2tutorials/1-ROS_basic_concepts_and_development_tools/index.html

Package adapted from Jason O'Kane https://www.cse.sc.edu/~jokane/agitr/ 

## Usage

Run **turtlesim_node** and move the turtle randomly by the velocities published by the **pubvel** node:

```
$ros2 run turtlesim turtlesim_node
$ros2 run foolturtle pubvel

```

Then run **subpose** node to track the poses of the turtle:

```
$ros2 run foolturtle subpose

```
