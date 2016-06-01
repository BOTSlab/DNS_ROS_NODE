#include <stdio.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Twist.h>

#include "Vector2d.h"

using namespace std;

class Controller{
public:
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // The arguments passed to the constructor are:
  // normal = Used to construct a vector perpendicular to the line segment
  // avoidanceAngle = If a neighbour bearing vector has a dot product greater
  //                  than this value, then the neighbour is in the dead 
  //                  ahead sensing region.
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Controller(float normalBearing = 0., float avoidance = 0.95){
    formationNormal = new Vector2d(normalBearing);
    avoidanceAngle = avoidance;

    cmdVelVector = new Vector2d();
  }
  
  ~Controller(){
    printf("Deleting Controller...");
    // Delete other pointers

    printf("DONE!\n");
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // This function is called by the ROS node once a cycle. It determines the
  // desired velocity vector based on the latest sensor data.
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  void UpdateResponse(){
    // Reset boolean flags
    robotDeadAhead = false;
    robotAhead = false;
    robotBehind = false;

    // Used to determine largest projection
    largestProjection = 0.;

    // Loop over all neighbour bearings and determine
    // 1. boolean flag values
    // 2. the bearing to the "target neighbour"
    for(unsigned int i = 0; i < neighbourBearings.size(); i++){
      // Calculate projection
      double p = formationNormal->DotProduct(neighbourBearings[i]);
      
      // Set boolean flag
      if(p > avoidanceAngle){ robotDeadAhead = true;}
      else if(p > 0.){ robotAhead = true;}
      else if(p < 0.){ robotBehind = true;}
      else{}

      // Find target neighbour (largest projection on to formation normal)
      if(p > largestProjection){
	largestProjection = p;
      }
    }

    //~~~~~~~~~~~~~~~This could be two different Functions~~~~~~~~~~~~

    // Now calculate the command velocity vector.
    delete cmdVelVector;
    if(robotDeadAhead){
      // This should be improved to rotate the vector depending on which 
      // side the robot in the dead ahead region is on. 
      cmdVelVector = formationNormal->GetPerpVector();
    }
    else if(robotBehind and ! robotAhead){
      cmdVelVector = formationNormal->MultiplyByScalar(-1.);
    }
    else if(robotAhead){
      cmdVelVector = formationNormal->MultiplyByScalar(largestProjection);
    }
    else{
      cmdVelVector = new Vector2d();
    }
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // This function returns a geomtry_msgs::Twist which encodes the desire
  // velocity
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  geometry_msgs::Twist GetTwist(){
    geometry_msgs::Twist aTwist;

    // Use the norm of the desired velocity for twist.linear.x
    aTwist.linear.x = cmdVelVector->GetNorm();

    // Use the difference between current heading and desire heading
    // for twist.angular.z
    double deltaHeading = currentHeading - cmdVelVector->GetBearing();
    aTwist.angular.z = deltaHeading;      

    return aTwist;    
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
  // These functions are called by the callback functions of the node
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  void UpdateNeighbourBearings(vector<double> bearingVector){
    // Use the bearingVector to create a vector of Vectors. This will make
    // determining which sensing region the neighbour is in easier. The 
    // bearings passed to the function are in the local frame of the robot.
    // These have to be converted to the global frame provided by the compass.
    
    // First clear the previous bearings
    neighbourBearings.clear();
    
    // Now loop through the bearings pass to the function.
    for(unsigned int i = 0; i < bearingVector.size(); i++){
      double aBearing = bearingVector[i];
      
      // Conversion to global frame.
      aBearing -= currentHeading;
      
      // Create a Vector2d and add it the the neighbourBearings vector.
      neighbourBearings.push_back(new Vector2d(aBearing));
    }
  }

  void UpdateHeading(double aHeading){
    currentHeading = aHeading;
  }

private:

  // Variables set by the constructor
  Vector2d* formationNormal; // Perpendicular to line segment
  double avoidanceAngle; // Defines size of deadAhead region

  // Variables set by sensor data
  double currentHeading;
  vector<Vector2d*> neighbourBearings;

  // Variables used to determine the behaviour 
  double largestProjection;
  bool robotAhead;
  bool robotDeadAhead;
  bool robotBehind;

  // Velocity command
  Vector2d * cmdVelVector;
  
};

