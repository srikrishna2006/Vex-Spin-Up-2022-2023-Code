#include <vex.h>
#include "Pure_Persuit.h"
#include <iostream>


pp::pp(Odometry *O) {
  odometry = O;
  newPoints = {};
  weightSmooth = 0.90;
  weightData = 1 - weightSmooth;
  tolerance = 0.1;
  rPos = {odometry->posX, odometry->posY};
  rAngle = O->currentThetaRadians(); 
  lookAheadP = {0, 0};
  lookAheadDist = 5.0;
  trackWidth = 0;
}

std::vector<Vector> pp::getNewPoints() 
{ 
  return newPoints; 
}
void pp::setLookAheadDist(double dist)
{
  lookAheadDist = dist;
}
void pp::setTrackWidth(double width)
{
  trackWidth = width;
}
double pp::getLookAhead()
{
  return lookAheadDist;
}

void pp::injection(std::vector<Vector> &v) 
{
  double spacing = 3.0;
  Vector lastPoint;

  for (int i = 0; i < v.size() - 1; i++) 
  {
    Vector firstPoint = v[i];
    lastPoint = v[i + 1];
    Vector vector = lastPoint.subtraction(firstPoint);
    int numPointsThatFit = ceil(vector.magnitude() / spacing);
    vector = vector.normalize();
    vector = {vector.getX() * spacing, vector.getY() * spacing};
    for (int j = 0; j < numPointsThatFit; j++) {
      newPoints.push_back(firstPoint.addition({vector.getX() * j, vector.getY() * j}));
    }
  }
  newPoints.push_back(lastPoint);
  for (int i = 0; i < newPoints.size(); i++) 
  {
    printf("injection: %f, %f,\n", newPoints[i].getX(), newPoints[i].getY());
  }
}
void pp::smoothing() 
{
  std::vector<Vector> newPath = newPoints;
  double change = tolerance;
  while (change >= tolerance) 
  {
    change = 0.0;
    for (int i = 1; i < newPoints.size() - 1; i++) 
    {
      double aux = newPoints[i].getX();
      double aux2 = weightData * (newPath[i].getX() - newPoints[i].getX()) + weightSmooth * (newPoints[i - 1].getX() + newPoints[i + 1].getX() - 2 * newPoints[i].getX());
      newPoints[i].setX(aux + aux2);
      change += fabs(newPoints[i].getX() - aux);

      double aux3 = newPoints[i].getY();
      double aux4 =
      weightData * (newPath[i].getY() - newPoints[i].getY()) +
      weightSmooth * (newPoints[i - 1].getY() + newPoints[i + 1].getY() - 2 * newPoints[i].getY());
      newPoints[i].setY(aux3 + aux4);
      change += fabs(newPoints[i].getY() - aux3);
    }
  }
  for (int i = 0; i < newPoints.size(); i++) 
  {
    printf("smooth: %f, %f,\n", newPoints[i].getX(), newPoints[i].getY());
  }
}

double pp::velAtPoint() 
{
  double dist;
  double a = 2.0;
  double vel = 0;
  for (int i = newPoints.size() - 1; i >= 0; i--) {
    Vector v = newPoints[i + 1];
    dist = v.distBetweenVectors(newPoints[i]);
    vel = sqrt(pow(vel, 2) + 2 * a * dist);
  }
  return vel;
}
Vector pp::closest() 
{
  double closestPoint = 0;
  double minDist = 10000000;
  Vector closestVector = {};
  for (int i = 0; i < newPoints.size() - 1; i++) 
  {
    closestPoint = rPos.distBetweenVectors(newPoints[i]);
    if (closestPoint < minDist) {
      minDist = closestPoint;
      closestVector = newPoints[i];
    }
  }
  //printf("closest point: %f, %f,\n", closestVector.getX(),closestVector.getY());
  return closestVector;
}
void pp::lookAheadPoint() 
{
  double tval = 0;
  Vector E;
  Vector d;
  Vector C = rPos;
  for (int i = newPoints.size() - 1; i > 0; i--) 
  {
    Vector L = newPoints[i];
    E = newPoints[i - 1];
    // printf("newpoints %f %f, %d, %f, %f, %f, %f\n", E.getX(), E.getY(), i, L.getX(), L.getY(), C.getX(), C.getY());
    d = L.subtraction(E);         // Direction vector from start to end
    Vector f = E.subtraction(C); // Vector from robotPos to ray start
    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - pow(lookAheadDist, 2);
    double discriminant = pow(b, 2) - 4 * a * c;
    // printf("a: %f, b: %f, c: %f, d: %f\n", a, b, c, discriminant);
    // printf("d: (%f, %f), f: (%f,%f)\n", d.getX(),d.getY(),f.getX(),f.getY());
    if (discriminant < 0) 
    {
      // no intersection
      //printf("returning no int\n");
      tval = 0;
    } 
    else 
    {   
      discriminant = sqrt(discriminant);
      double t1 = (-b - discriminant) / (2 * a);
      double t2 = (-b + discriminant) / (2 * a);
      if (t1 >= 0 && t1 <= 1) 
      {
        // return t1 intersection
       // printf("returning t1 int\n");
        tval = t1;
        break;
      }
      if (t2 >= 0 && t2 <= 1) 
      {
        // return t2 intersection
       // printf("returning t2 int\n");
        tval = t2;
        break;
      }
      // otherwise, no intersection
      //printf("returning no int after conditionals\n");
      //tval = -1;
    }

  }
  if (tval < 0)
  {
    lookAheadP = closest();
    printf("Closest %f %f, %f\n", lookAheadP.getX(), lookAheadP.getY(), tval);
    return;
  }
  Vector lookAhead = {E.getX() + (tval * d.getX()), E.getY() + (tval * d.getY())};
  printf("lookahead point: (%f, %f, %f, %f, %f)\n", lookAhead.getX(), lookAhead.getY(), E.getX(), E.getY(), tval);
  lookAheadP = lookAhead;
}

double pp::sign(double val) 
{
  if (val > 0) 
  {
    return 1;
  }
  if (val < 0) 
  {
    return -1;
  } 
  else
    return 1;
}

double pp::curvature() 
{  
  double rTheta = M_PI/2 - this->rAngle;
  double side = sign(sin(rTheta) * (lookAheadP.getX() - rPos.getX()) - cos(rTheta) * (lookAheadP.getY() - rPos.getY()));
  
  double a = -tan(rTheta);
  double b = 1;
  double c = tan(rTheta) * rPos.getX() - rPos.getY(); // robot x and y. O.posX and O.posY
  double x = fabs(a * lookAheadP.getX() + b * lookAheadP.getY() + c) /(sqrt(pow(a, 2) + pow(b, 2)));
  //printf("side %f, %f, %f, %f, %f\n", side, a, b, c, x);

  double curvature = ((2 * x) / (pow(lookAheadDist, 2)));
  //printf("curv %f\n", curvature);
  //printf("%f,%f,%f\n", rPos.getX(),rPos.getY(),rAngle);
  return curvature * side;
}

void pp::move()
{
  double tVel = 10; // target robot vel 
  double curvature = this->curvature();
  double leftVel = tVel * (2 - curvature * trackWidth)/2;
  double rightVel = tVel * (2 + curvature * trackWidth)/2;
  Vector lastPoint = newPoints[newPoints.size() - 1];
  double distanceToEnd = 100000;
  printf("pos -> (%f, %f, %f) \n", rPos.getX(), rPos.getY(), rAngle);
  while(distanceToEnd > lookAheadDist)
  {
    tVel = 20; // target robot vel 
    lookAheadPoint();
    curvature = this->curvature();
    leftVel = tVel * (2 + curvature * trackWidth)/2;
    rightVel = tVel * (2 - curvature * trackWidth)/2;
    //printf("(%f, %f) vel\n", leftVel, rightVel);
    LB.spin(forward,leftVel,percent);
    LF.spin(forward,leftVel,percent);
    RB.spin(forward,rightVel,percent);
    RF.spin(forward,rightVel,percent);
    odometry->update();
    rPos.setX(odometry->posX);
    rPos.setY(odometry->posY);
    rAngle = odometry->currentThetaRadians(); 
    distanceToEnd = lastPoint.distBetweenVectors(rPos);
    printf(" (%f,%f),curv: %f\n", odometry->posX,odometry->posY,curvature);
    wait(100, msec);
    //break;
    // for (int i = 0; i < newPoints.size(); i++) 
    // {
    //   printf("%f, %f,\n", lookAheadP.getX(), lookAheadP.getY());
    // }

  }
    LB.stop();
    RB.stop();
    LF.stop();
    RF.stop();
  
}


