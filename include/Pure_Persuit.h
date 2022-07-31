#pragma once
#include "Vector_Math.h"
#include "Odometry.h"
#include <vector>
class pp
{
  private:
    Odometry *odometry;
    std::vector<Vector> newPoints;
    double weightSmooth;
    double weightData;
    double tolerance;
    double radius;
    double rAngle;
    Vector rPos;
    Vector lookAheadP;
    double sign(double val);
    double lookAheadDist;
    double trackWidth;
  public:
    pp(Odometry *O);
    void setTrackWidth(double width);
    void setLookAheadDist(double dist);
    double getLookAhead();
    void injection(std::vector<Vector>&v);
    void smoothing();
    std::vector<Vector> getNewPoints();
    double velAtPoint();
    double curvature();
    void lookAheadPoint();
    Vector closest();
    void move();

    
};