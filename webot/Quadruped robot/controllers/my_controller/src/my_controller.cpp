// File:          my_controller.cpp
// Date:
// Description:
// Author:  baolan
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <motionControl.cpp>
#include <ctime>
#include <fstream>
#define PI 3.1415926

// All the webots classes are defined in the "webots" namespace
using namespace webots;
MotionControl mc;
int main(int argc, char **argv) {
    Robot *robot = new Robot();
    Keyboard kb;
    kb.enable(1000);
    int key;
    float timePeriod = 0.01;
    mc.MotionContr(timePeriod, robot);

    Camera *camera;
    camera=robot->getCamera("camera");
    camera->enable(TIME_STEP);

    int timeStep = (int)robot->getBasicTimeStep();

    Matrix<float, 4, 3> initPos;
    initPos<<  0.0, 0.0, -0.24, 0.0, 0.0, -0.24, 0.0, 0.0, -0.24, 0.0, 0.0, -0.24;
    mc.setInitPos(initPos);
    Vector<float, 3> tCV;
    tCV<< 0.0, 0.0, 0.0;
    //w:87,s:83,a:65,d:68，q:81,e:69
    mc.setCoMVel(tCV);
    char StateVal;
    StateVal = mc.stateval[0];

    ofstream oFile;
    oFile.open("force.csv",ios::out | ios::trunc);
  while (robot->step(timeStep) != -1)
  {
    key = kb.getKey();
    if (key == 87)
    {
      tCV(0) = tCV(0) + 0.01;
    }
    else if (key == 83)
    {
      tCV(0) = tCV(0) - 0.01;
    }
    else if (key == 65)
    {
      tCV(1) = tCV(1) + 0.01;
    }
    else if (key == 68)
    {
      tCV(1) = tCV(1) - 0.01;
    }
    else if(key == 81)
    {
      tCV(2) = tCV(2) + 0.01;
    }
    else if(key == 69)
    {
      tCV(2) = tCV(2) - 0.01;
    }
    else
    {
      tCV = tCV;
    }
    mc.Sensor_update();
    mc.setCoMVel(tCV);
    mc.forwardKinematics();
    mc.jacobians();
    mc.state_judgement();
    // clock_t startTime,endTime;
    // startTime = clock();//计时开始
    switch(StateVal)
    {
      case 'a'://
         mc.State_start();
         if (mc.Start_time >= 0.20)
         {
           StateVal = mc.stateval[1];
           // mc.Start_time = 0.0;
           mc.fly_time = 0.0;
         }
         cout << "a"<<endl;
         break;
      case 'b':// 2 leg stance
        mc.stance_VMC();
        mc.swing_VMC();
        mc.Setjoint();
        cout << "time:   "<< mc.fly_time <<endl;
        // if (mc.fly_time > mc.T - 0.0005 && mc.state_val == 1)
        //   StateVal = mc.stateval[3];
        // else if (mc.fly_time > mc.T - 0.0005 && mc.state_val == 2)
        //   StateVal = mc.stateval[2];
        cout << "b"<< endl;
        break;
      case 'c'://3 leg stance
        mc.stance_VMC_3leg();
        mc.swing_VMC_3leg();
        mc.Setjoint();
        if (mc.fly_time > mc.T - 0.0005 || mc.state_val == 1)
          StateVal = mc.stateval[3];
        cout << "c"<< endl;
        break;
      case 'd'://4 leg stance
        mc.stance_VMC_4leg();
        mc.Setjoint();
        if (mc.swingFlag == 0)
        {
          mc.swingFlag = 1;
          mc.fly_time = 0.0;
        }
        else
        {
          mc.swingFlag = 0;
          mc.fly_time = 0.0;
        }
        StateVal = mc.stateval[1];
        oFile << mc.leg_force[0]<<","<< mc.leg_force[1]<<","<< mc.leg_force[2]<<","<< mc.leg_force[3]<<","<< mc.leg_force[4]<<","<< mc.leg_force[5]<<","<< mc.leg_force[6]<<","<< mc.leg_force[7]<<","<< mc.leg_force[8]<<","<< mc.leg_force[9]<<","<< mc.leg_force[10]<<","<< mc.leg_force[11]<< endl;
        cout << "d" <<endl;
        break;
      default:
      break;
    }
    cout << "v:   "<<tCV.transpose()<<endl;
  }
oFile.close();
delete robot;
return 0;
}
