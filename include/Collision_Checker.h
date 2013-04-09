#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Collision_Checker
{
public:

    /**
* Constructor for the Collision_Checking class
*/
    Collision_Checker();

    /**
* Deconstructor for the Collision_Checking class
*/
    ~Collision_Checker();

    void initCollisionChecker();
    /**
* Checks for collision of the hands with the torso.
* If the goal location is inside the boundary ellipse defined by parameters a (minor axis)
* and b (major axis) then the x,y location gets mapped to the surface of the ellipse
*/
    void checkSelfCollision(Eigen::Isometry3d &goal);

private:

    double xMinor; // minor axis parameter for collision boundary ellipse
    double yMajor; // major axis parameter for collision boundary ellipse
    double collisionLimit; // distance to point on ellipse
};

#endif // COLLISION_CHECKER_H
