#include "Vector_Math.h"

Vector::Vector()
{

}

Vector::Vector(double x, double y)
{
  p1 = x;
  p2 = y;
}
void Vector::setX(double x)
{
  p1 = x;
}
void Vector::setY(double y)
{
  p2 = y;
}
double Vector::getX()
{
  return p1;
}
double Vector::getY()
{
  return p2;
}

double Vector::magnitude()
{

  double mag = sqrt(pow(p1,2) + pow(p2,2));
  return mag;
}

Vector Vector::addition(Vector v)
{

  return Vector(v.getX() + getX(),v.getY() + getY());
}

Vector Vector::subtraction(Vector v)
{

  return Vector(getX() - v.getX(),getY() - v.getY());
}

double Vector::dot(Vector v)
{
  return v.getX()*getX() + v.getY()*getY();
}

Vector Vector::normalize()
{
  return Vector(p1/magnitude(),p2/magnitude());
}
double Vector::distBetweenVectors(Vector v)
{
  return sqrt(pow(v.getX() - getX(),2) + pow(v.getY() - getY(),2));
  
}
int Vector::length()
{
  return 1;
}





