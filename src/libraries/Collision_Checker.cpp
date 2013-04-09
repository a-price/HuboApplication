#include<Collision_Checker.h>

Collision_Checker::Collision_Checker()
{
    initCollisionChecker();
}

Collision_Checker::~Collision_Checker()
{
}

void Collision_Checker::initCollisionChecker()
{
    xMinor = 0.23; // minor axis parameter for collision boundary ellipse
    yMajor = 0.29; // major axis parameter for collision boundary ellipse
    collisionLimit = 1.0; // distance to point on ellipse
}

void Collision_Checker::checkSelfCollision(Eigen::Isometry3d &goal) {

    double x = goal(0,3); // get x position of goal
    double y = goal(1,3); // get y position of goal

    // get distance to point x,y using ellipse equation
    double dist = (x*x)/(xMinor*xMinor) + (y*y)/(yMajor*yMajor);

    // check if dist is inside boundary ellispe defined by xMinor and yMajor
    if (dist < collisionLimit)
    {
        std::cout << "Goal of " << "(" << x << "," << y << ")" << " causes a collision\n";
        printf("\a");
        // adjust x and y to be on at the point on the collision boundary
        // on the opposite side of x,y from the origin
        double scale = sqrt(collisionLimit) / sqrt(dist);
        x = scale * x;
        y = scale * y;
        // redefine goal x,y
        goal(0,3) = x;
        goal(1,3) = y;
    }
}


