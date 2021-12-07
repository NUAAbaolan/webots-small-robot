#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <fstream>
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>

using namespace std;
using namespace Eigen;
using namespace webots;

#ifndef MotionControl_H_H
#define MotionControl_H_H
namespace webots{
class PositionSensor;
class Motor;
class TouchSensor;
class InertialUnit;
}
class MotionControl
{
    public:
        //传感器引用
        char *robot;
        char positionName[12][22] = {"LFL0_position sensor", "LFL1_position sensor", "LFL2_position sensor", "RFL0_position sensor", "RFL1_position sensor", "RFL2_position sensor", "LHL0_position sensor", "LHL1_position sensor", "LHL2_position sensor", "RHL0_position sensor", "RHL1_position sensor", "RHL2_position sensor"};
        char motorName[12][22] = {"LFL0_rotational motor", "LFL1_rotational motor", "LFL2_rotational motor", "RFL0_rotational motor", "RFL1_rotational motor", "RFL2_rotational motor", "LHL0_rotational motor", "LHL1_rotational motor", "LHL2_rotational motor", "RHL0_rotational motor", "RHL1_rotational motor", "RHL2_rotational motor"};
        char touchsensorName[4][16] = {"LF_touch sensor", "RF_touch sensor", "LH_touch sensor", "RH_touch sensor"};
        char stateval[4] = {'a', 'b', 'c', 'd'};//状态机参数
        PositionSensor *Ps[12];
        Motor *Tor[12];
        TouchSensor *Ts[12];
        InertialUnit *imu;
        //各个函数具体调用
        float timePeriod;// The time of one period
        Vector<float, 3> targetCoMVelocity;//X, Y , alpha c in world cordinate
        float L1, L2, L3;  // The length of L
        float width, length;
        int key;// keyboard
        Vector<float, 3> tCV; //target velocity of x,y,taoz
        Vector<float, 12> jointPresentPos;  // present joint 0-11
        Matrix<float, 4, 3> legCmdPos;//主要用于初始化机器人姿态，使得机器人能够弯曲站立在表面上
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Vector<float, 3> legPresentPos_target;
        Vector<float, 12> jointPresentVel;  // present motor 0-11
        Vector<float, 3> imu_num;
        Vector<float, 3> imuVel;
        Vector<float, 4>state;//present leg touch sensor
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Matrix<float, 4, 3> leg2CoMPrePos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 9>jacobian;
        int state_val;//状态机参数
        //逆运动学参数，现在还没用到，需要PD补偿时，才会用到
        float jointCmdPos[12];  // command motor 0-11
        float jointCmdPosLast[12];
        float jointCmdVel[12];
        Vector<float, 12> jacobian_torque;
        float Start_time = 0.0;

        float target_taoz = 0.0;
        float fly_time;
        float T = 0.3;
        float H = 0.1;
        float G = 3;
        float kx = 95.5;
        float ky = 85;
        float kz = 125;
        float dx = 23;
        float dy = 21.5;
        float dz = 18.5;
        float k_taox = 38;//
        float k_taoy = 78;//
        float k_taoz = 30;
        float d_taox = 0.66;//
        float d_taoy = 3.53;//
        float d_taoz = 5;

        float swingfx_kp = 1;
        float swingfy_kp = 1;
        float swingfz_kp = 1;

        float swingfx_kd = 1;
        float swingfy_kd = 1;
        float swingfz_kd = 1;

        float swinghx_kp = 1;
        float swinghy_kp = 1;
        float swinghz_kp = 1;

        float swinghx_kd = 1;
        float swinghy_kd = 1;
        float swinghz_kd = 1;
        int swingFlag = 0; // 0 is 2 and 3 swing ;1 is 1 and 4 swing
        Vector<int, 3> leg;//三足支撑时，支撑相集合
        Vector<float, 12> leg_force;

        void MotionContr(float tP, Robot *robot);
        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 3> tCV);
        void forwardKinematics();
        void Sensor_update();
        void jacobians();
        void state_judgement();
        void inverseKinematics();   // standing state
        void State_start();
        void stance_VMC();
        void swing_VMC();
        void stance_VMC_3leg();
        void swing_VMC_3leg();
        void stance_VMC_4leg();
        void Setjoint();
};
#endif