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
    float timePeriod = 0.01;
    mc.MotionContr(timePeriod, robot);

    Camera *camera;
    camera=robot->getCamera("camera");
    camera->enable(TIME_STEP);

    int timeStep = (int)robot->getBasicTimeStep();

    Matrix<float, 4, 3> initPos;
    initPos<<  0.0, 0.0, -0.24, 0.0, 0.0, -0.24, 0.0, 0.0, -0.24, 0.0, 0.0, -0.24;
    mc.setInitPos(initPos);
    mc.tCV<< 0.0, 0.0, 0.0;
    //w:87,s:83,a:65,d:68，q:81,e:69
    mc.setCoMVel(mc.tCV);
    char StateVal;
    StateVal = mc.stateval[0];

    ofstream oFile;
    oFile.open("force.csv",ios::out | ios::trunc);
  while (robot->step(timeStep) != -1)
  {
    mc.key = kb.getKey();
    mc.Sensor_update();
    mc.setCoMVel(mc.tCV);
    mc.forwardKinematics();
    mc.jacobians();
    mc.state_judgement();
    switch(StateVal)
    {
      case 'a'://
         mc.State_start();
         if (mc.Start_time >= 0.20)
         {
           StateVal = mc.stateval[1];
           mc.fly_time = 0.0;
         }
         cout << "a"<<endl;
         break;
      case 'b':// 2 leg stance
        mc.stance_VMC();
        mc.swing_VMC();
        mc.Setjoint();
        cout << "time:   "<< mc.fly_time <<endl;
        if (mc.fly_time >mc.T - 0.0005 && mc.state_val == 1)
          StateVal = mc.stateval[3];
        else if (mc.fly_time > mc.T - 0.0005 && mc.state_val == 2)
          StateVal = mc.stateval[2];
        else if (mc.fly_time > mc.T - 0.0005)
          StateVal = mc.stateval[3];
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
        static int temp_num = 0;
        if (temp_num == 5)
        {
          StateVal = mc.stateval[1];
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
          temp_num = 0;
        }
        temp_num += 1;
        oFile << mc.leg_force[0]<<","<< mc.leg_force[1]<<","<< mc.leg_force[2]<<","<< mc.leg_force[3]<<","<< mc.leg_force[4]<<","<< mc.leg_force[5]<<","<< mc.leg_force[6]<<","<< mc.leg_force[7]<<","<< mc.leg_force[8]<<","<< mc.leg_force[9]<<","<< mc.leg_force[10]<<","<< mc.leg_force[11]<< endl;
        cout << "d" <<endl;
        break;
      default:
      break;
    }
    cout << "v:   "<<mc.tCV.transpose()<<endl;
  }
oFile.close();
delete robot;
return 0;
}
