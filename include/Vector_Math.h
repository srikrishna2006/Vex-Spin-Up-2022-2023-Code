#pragma once
#include "vex.h"
class Vector
{
  private:
  double p1;
  double p2;

  public:
  Vector();
  Vector(double x, double y);
  void setX(double x);
  void setY(double y);
  double getX();
  double getY();
  double magnitude();
  Vector addition(Vector v);
  Vector subtraction(Vector v);
  double dot(Vector v);
  Vector normalize();
  double distBetweenVectors(Vector v);
  int length();
};