#include <vex.h>
#include <Odometry.h>

Odometry::Odometry()
    {
      this->posX = 0;
      this->posY = 0;
      this->theta = 0;
      this->turnsL = 0;
      this->turnsR = 0;
    }
    
    double Odometry::currentTheta()
    {
      return this->theta * 180 / M_PI;
    }

    double Odometry::currentThetaRadians()
    {
      double theta = this->theta;
      if (theta > M_PI)
      {
        theta = -2 * M_PI + this->theta;
      }
      else if (theta < -M_PI)
      {
        theta = 2 * M_PI + this->theta;
      }
      return theta;
    }
  

    void Odometry::update()
    {
      double ld = 0;
      double rd = 0;
      double dtheta = 0;
      double dist = 0;
      double ROTl = Left.position(turns);
      double ROTr = Right.position(turns);
      // printf("rotation %f, %f\n", ROTl, ROTr);
      double currX = 0;
      double currY = 0; 

      ROTl -= this->turnsL;
      ROTr -= this->turnsR;


      ld= M_PI*ROTl*WHDIA;

      rd = M_PI*ROTr*WHDIA;

      dtheta = (ld - rd)/(2*RW);

      dist = (ld + rd)/2;

      currY = dist*cos(this->theta+dtheta/2);

      currX = dist*sin(this->theta+dtheta/2);
      
      this->posX += currX;
      this->posY += currY;
      this->theta += dtheta;
      if (this->theta > 2*M_PI)

        this->theta -= (2*M_PI);

      if (this->theta < -(2*M_PI))

        this->theta += (2*M_PI);

      this->turnsR += ROTr;
      this->turnsL += ROTl;
    }