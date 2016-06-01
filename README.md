This repo contains a ros node which implements the Dynamic Neighbour Selection pattern formation algorithm.

The node subscribes to two topics:
1. /neighbourBearings which is of type std_msgs/Float64MultiArray (an array of doubles )
2. /compass which of type std_msgs/Float64 (a single double)

The /neighbourBearings topic publishes the bearing (angle) to the detected robot (measure via the omni-directional camera) with respect to the robots heading. The bearing should be to the detected blobs center of mass. The bearings should cover the range [pi, -pi], with CCW bearing being positive and CW bearings being negitive.

The /compass topic publishes the current heading of the robot with respect to North as measure by the on board compass.

The node publishes to the the topic /cmd_vel with a message of type geometry_msgs/Twist. 

The code is broken into three files DynamicNeighbourSelection.cpp, Controller.h, and Vector2d.h.

1. DynamicNeighbourSelection.cpp contains all the ROS related code as well as an instance of the Controller class.
2. Controller.h contains the actual implemenation of the DNS algorithm.
3. Vector2d.h is a helper class that does basic 2d vector calculations. It is used only by the Controller class.

There is an additional file test.cpp which is just a test of the Vector2d.h calculations.

Currently the DynamicNeighbourSelection node has a statically defined formation normal, and avoidanceAngle. These values should be defined in a launch file when using roslaunch, or by command line arguments if starting the node using rosrun.



