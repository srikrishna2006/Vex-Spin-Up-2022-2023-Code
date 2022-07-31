#pragma once
#define WHDIA 3.25
#define RW 6.25


class Odometry 
{   

  public:

    double posX;
    double posY;
    double theta;
    double turnsL;
    double turnsR;

  

    double currentTheta();
    double currentThetaRadians();


    void update();
    
    Odometry();
};

