#include <motiondefine.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <math.h>
#include <qpOASES.hpp>
#define PI 3.1415926
#define TIME_STEP 10

using namespace std;
using namespace Eigen;

void MotionControl::MotionContr(float tP, Robot *robot)
{
    timePeriod = tP;
    targetCoMVelocity << 0.0, 0.0, 0.0;
    L1 = 0.132;
    L2 = 0.138;
    L3 = 0.047;
    width = 0.087;
    length = 0.174;
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
    legPresentPos_target << 0, 0, -0.24;
    for (int i=0; i<12; i++)
    {
      Ps[i] = robot->getPositionSensor(positionName[i]);
      Ps[i]->enable(TIME_STEP);
      Tor[i] = robot->getMotor(motorName[i]);
    }
    for (int i=0; i<4; i++)
    {
      Ts[i] = robot->getTouchSensor(touchsensorName[i]);
      Ts[i]->enable(TIME_STEP);
    }
    imu = robot->getInertialUnit("inertial unit");
    imu->enable(TIME_STEP);
}
void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    legPresentPos = initPosition;
    legCmdPos = initPosition;
}
void MotionControl::Sensor_update()
{
    Vector<float, 12> jointLastPos;
    for (int i = 0; i < 12; i++)
    {
        jointLastPos(i) = jointPresentPos(i);
    }
    Vector<float, 3> Lastimu_num;
    for (int i = 0; i < 3; i++)
    {
        Lastimu_num(i) = imu_num(i);
    }
    
    for (int i=0; i<12; i++)
    {
        jointPresentPos(i) = Ps[i]->getValue();
    }
    float temp_touch1 = Ts[0]->getValue();
    float temp_touch2 = Ts[1]->getValue();
    float temp_touch3 = Ts[2]->getValue();
    float temp_touch4 = Ts[3]->getValue();
    state << temp_touch1, temp_touch2, temp_touch3, temp_touch4;
    imu_num << imu->getRollPitchYaw()[0] + 0.5 * PI, - imu->getRollPitchYaw()[1], imu->getRollPitchYaw()[2] - 0.5 * PI;
    cout << "imu:   "<< imu_num.transpose()<<endl;
    for (int i = 0; i < 12; i++)
    {
        jointPresentVel(i) = (jointPresentPos(i) - jointLastPos(i))/ timePeriod;
    }

    for (int i = 0; i < 3; i++)
    {
        imuVel(i) = (imu_num(i) - Lastimu_num(i)) / timePeriod;
    }
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
}
void MotionControl::forwardKinematics()
{
    float joint_pres_pos[4][3];
    joint_pres_pos[0][0] = jointPresentPos[0];
    joint_pres_pos[0][1] = jointPresentPos[1];
    joint_pres_pos[0][2] = jointPresentPos[2];
    joint_pres_pos[1][0] = jointPresentPos[3];
    joint_pres_pos[1][1] = jointPresentPos[4];
    joint_pres_pos[1][2] = jointPresentPos[5];
    joint_pres_pos[2][0] = jointPresentPos[6];
    joint_pres_pos[2][1] = jointPresentPos[7];
    joint_pres_pos[2][2] = jointPresentPos[8];
    joint_pres_pos[3][0] = jointPresentPos[9];
    joint_pres_pos[3][1] = jointPresentPos[10];
    joint_pres_pos[3][2] = jointPresentPos[11];
    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = -L1 * sin(joint_pres_pos[leg_nums][1]) - L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,1) = L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) + L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,2) = -L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) - L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        leg2CoMPrePos(leg_nums,0) = shoulderPos(leg_nums,0) + legPresentPos(leg_nums,0);
        leg2CoMPrePos(leg_nums,1) = shoulderPos(leg_nums,1) + legPresentPos(leg_nums,1);
        leg2CoMPrePos(leg_nums,2) = legPresentPos(leg_nums,2);
    }
}
void MotionControl::jacobians()
{
    jacobian(0 ,0) = 0;
    jacobian(0 ,1) =  -L1 * cos(jointPresentPos(1)) -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,2) =  -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,3) = L1 * cos(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,4) = -L1 * sin(jointPresentPos(0)) * sin(jointPresentPos(1)) - L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,5) = -L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,6) = L1 * sin(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * sin(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,7) = L1 * cos(jointPresentPos(0)) * sin(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,8) = L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));

    jacobian(1 ,0) = 0;
    jacobian(1 ,1) =  -L1 * cos(jointPresentPos(4)) -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,2) =  -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,3) = L1 * cos(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,4) = -L1 * sin(jointPresentPos(3)) * sin(jointPresentPos(4)) - L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,5) = -L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,6) = L1 * sin(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * sin(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,7) = L1 * cos(jointPresentPos(3)) * sin(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,8) = L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));

    jacobian(2, 0) = 0;
    jacobian(2 ,1) =  -L1 * cos(jointPresentPos(7)) -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,2) =  -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,3) = L1 * cos(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,4) = -L1 * sin(jointPresentPos(6)) * sin(jointPresentPos(7)) - L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,5) = -L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,6) = L1 * sin(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * sin(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,7) = L1 * cos(jointPresentPos(6)) * sin(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,8) = L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));

    jacobian(3 ,0) = 0;
    jacobian(3 ,1) =  -L1 * cos(jointPresentPos(10)) -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,2) =  -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,3) = L1 * cos(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,4) = -L1 * sin(jointPresentPos(9)) * sin(jointPresentPos(10)) - L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,5) = -L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,6) = L1 * sin(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * sin(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,7) = L1 * cos(jointPresentPos(9)) * sin(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,8) = L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            if (jacobian(i, j) < -10 || jacobian (i, j) > 10)
            {
                jacobian(i ,j) = 0.0;
                cout << "jacobian"<<endl;
            }
        }
    }
}
void MotionControl::state_judgement()
{
    if (state.sum() == 4)
        state_val = 1;
    else if(state.sum() == 3)
        state_val = 2;
    else if(state.sum() == 2)
        state_val = 3;
    else
        cout << "one leg stance"<< endl;
}
void MotionControl::inverseKinematics()
{
    float jo_ang[4][3] = {0};
    static int times = 0;
    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdPosLast[joints] = jointPresentPos[joints];
            jointCmdPos[joints] = jointPresentPos[joints];
        }
    }
    for(int leg_num = 0; leg_num < 4; leg_num++)
    {
        float x = legCmdPos(leg_num,0);
        float y = legCmdPos(leg_num,1);
        float z = legCmdPos(leg_num,2);
        float theta0 = atan (- y / z);
        float theta2 = - acos ((pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2)) ; 
        float cos_theta1 = ((sin (theta2) * y - cos (theta0) * z) * (L1 + L2 * cos (theta2)) - x * L2 * sin(theta2)) / (pow((sin (theta2) * y - cos (theta0) * z), 2) + pow(x, 2));
        float sin_theta1 = ((L1 + L2 * cos (theta2)) * x + (sin (theta2) * y - cos (theta0) * z) * L2 * sin(theta2)) / (-pow((sin (theta2) * y - cos (theta0) * z), 2) - pow(x, 2));
        float theta1 = atan (sin_theta1/ cos_theta1);
        jo_ang[leg_num][0] = theta0;
        jo_ang[leg_num][1] = theta1;
        jo_ang[leg_num][2] = theta2;
    }
    jointCmdPos[0] = jo_ang[0][0];
    jointCmdPos[1] = jo_ang[0][1];
    jointCmdPos[2] = jo_ang[0][2];
    jointCmdPos[3] = jo_ang[1][0];
    jointCmdPos[4] = jo_ang[1][1];
    jointCmdPos[5] = jo_ang[1][2];
    jointCmdPos[6] = jo_ang[2][0];
    jointCmdPos[7] = jo_ang[2][1];
    jointCmdPos[8] = jo_ang[2][2];
    jointCmdPos[9] = jo_ang[3][0];
    jointCmdPos[10] = jo_ang[3][1];
    jointCmdPos[11] = jo_ang[3][2];
    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = (jointCmdPos[joints] - jointCmdPosLast[joints]) / timePeriod;
        }
    }
    else
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = 0;
        }
    }
    times++;
}
void MotionControl::Setjoint()
{
    for (int i= 0 ; i<12; i++)
    {
        if (jacobian_torque(i) < -2000 || jacobian_torque(i) > 2000)
        jacobian_torque(i) = 0.0;
    }
    Vector<float, 12> temp_jointCmdPos, temp_jointCmdVel;
    for(uint8_t joints=0; joints<12; joints++)
    {
        temp_jointCmdPos(joints) = jointCmdPos[joints];
        temp_jointCmdVel(joints) = jointCmdVel[joints];
    }
    Vector<float, 12> temp_motorCmdTorque;
    temp_motorCmdTorque = 0.0* (temp_jointCmdPos - jointPresentPos) + 0.0* (temp_jointCmdVel - jointPresentVel);
    jacobian_torque += temp_motorCmdTorque;
    for (int i = 0; i < 12; i++)
    {
        Tor[i]->setTorque(jacobian_torque(i));
    }
}
void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}
void MotionControl::State_start()
{
    inverseKinematics();
    for (int i= 0; i<12; i++)
    {
        Tor[i]->setPosition(jointCmdPos[i]);
    }
    Start_time += timePeriod;
}
void MotionControl::stance_VMC()
{
    target_taoz = target_taoz + targetCoMVelocity[2] * timePeriod;
    float tao_x = k_taox * (0.0 - imu_num[0]) + d_taox * (0 -imuVel(0));
    float tao_y = k_taoy * (0.0 - imu_num[1]) + d_taoy * (0 -imuVel(1));
    float tao_z = k_taoz * (target_taoz - imu_num[2]) + d_taoz * (targetCoMVelocity[2]  -imuVel(2));
    Matrix<float, 6, 6> A;
    Vector<float, 6> B;
    Vector<float, 3> presentCoMVelocity;
    if (swingFlag == 0)
    {
        float xf = leg2CoMPrePos(0, 0);
        float yf = leg2CoMPrePos(0, 1);
        float zf = leg2CoMPrePos(0, 2);
        float xh = leg2CoMPrePos(3, 0);
        float yh = leg2CoMPrePos(3, 1);
        float zh = leg2CoMPrePos(3, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 1,
             0, -zf, yf, 0, -zh, yh,
             zf, 0, -xf, zh, 0, -xh,
             -yf, xf, 0, -yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                       jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                       jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(9);
        temp_vel(1) = jointPresentVel(10);
        temp_vel(2) = jointPresentVel(11);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix*temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);
        float Fx = -kx * (0 - legPresentPos(3,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.24- legPresentPos(3,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * (0.0009 - legPresentPos(3, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    else
    {
        float xf = leg2CoMPrePos(1, 0);
        float yf = leg2CoMPrePos(1, 1);
        float zf = leg2CoMPrePos(1, 2);
        float xh = leg2CoMPrePos(2, 0);
        float yh = leg2CoMPrePos(2, 1);
        float zh = leg2CoMPrePos(2, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 1,
             0, -zf, yf, 0, -zh, yh,
             zf, 0, -xf, zh, 0, -xh,
             -yf, xf, 0, -yh, xh, 0;
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                       jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                       jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(6);
        temp_vel(1) = jointPresentVel(7);
        temp_vel(2) = jointPresentVel(8);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix * temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);
        float Fx = -kx * ( 0 - legPresentPos(2,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.24 - legPresentPos(2,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * ( 0.0009 - legPresentPos(2, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    // Matrix<float, 12, 6> temp_matrix;
    // float temp = 0.08;
    // temp_matrix.block(0,0,6,6) = A;
    // temp_matrix.block(6,0,6,6) = temp * MatrixXf::Identity(6, 6);
    // temp_matrix(8, 2) = 0.1 * temp_matrix(8, 2);
    // temp_matrix(11, 5) = 0.1 * temp_matrix(11, 5);
    // temp_matrix(6, 0) = 10 * temp_matrix(6, 0);
    // temp_matrix(9, 3) = 10 * temp_matrix(9, 3);
    // Vector<float, 12>temp_vector;
    // temp_vector.head(6) = B;
    // temp_vector.tail(6) << 0,0,0,0,0,0;
    // Vector<float, 6> temp_Force;
    // JacobiSVD<MatrixXf> svd(temp_matrix, ComputeThinU | ComputeThinV);
    // temp_Force = svd.solve(temp_vector);
    float alpha = 0.001;
    Matrix<float, 6, 6> temp_H;
    Vector<float, 6> temp_g;
    temp_H = 2 * A.transpose() * A + 2 * alpha * MatrixXf::Identity(6, 6);
    temp_g = - 2 * A.transpose() * B;
    Vector<float, 6> temp_Force;
    Matrix<float, 6, 6>jacobian_Matrix;
    USING_NAMESPACE_QPOASES
    real_t H[6*6];
    real_t g[6];
    for (int i = 0; i < 6; i++)
    {
        for (int j= 0; j<6; j++)
        {
            H[6 * i + j] = temp_H(i, j);
        }
        g[i] = temp_g(i);
    }
    float u = 4;
	real_t fA[8*6] = {u, 0, 1, 0, 0, 0,
                    -u, 0, 1, 0, 0, 0,
                    0, u, 1, 0, 0, 0,
                    0, -u, 1, 0, 0, 0,
                    0, 0, 0, u, 0, 1,
                    0, 0, 0, -u, 0, 1,
                    0, 0, 0, 0, u, 1,
                    0, 0, 0, 0, -u, 1};
    real_t lb[6] = {-40, -40, 0, -40, -40, 0};
    real_t ub[6] = {40, 40, 40, 40, 40, 40};
    real_t lbA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    real_t ubA[8] = {100000, 100000, 1000000, 1000000, 1000000, 100000, 100000, 100000};
	SQProblem example( 6,8 );
	int_t nWSR = 30;
	example.init( H,g,fA,lb,ub,lbA,ubA,nWSR);
	real_t xOpt[6];
	example.getPrimalSolution( xOpt );
    temp_Force << xOpt[0], xOpt[1], xOpt[2], xOpt[3], xOpt[4], xOpt[5];
    cout << "force:   "<< temp_Force.transpose() << endl;
    if (swingFlag == 0)
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                          jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                          jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                          jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                          jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.head(3) = temp_torque.head(3);
        jacobian_torque.tail(3) = temp_torque.tail(3);
        
    }
    else
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                          jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                          jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                          jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                          jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.segment(3, 6) = temp_torque;
    }
}
void MotionControl::swing_VMC()
{
    //足端轨迹的优化，使得足端轨迹能够越障，以及设计特定轨迹实现变姿态
    float S = T * targetCoMVelocity[0]/2;
    float t = fly_time ; 
    float x = S * (t/T-sin(2*PI*t/T)/(2*PI));
    float vx = S * (1/T-cos(2*PI*t/T)/T);
    float sgn;
    if ( t >= 0 && t < T/2)
    {
       sgn = 1;
    }
    else
    {
       sgn = -1;
    }
    float z = H * (sgn * (2*(t/T-sin(4*PI*t/T)/(4*PI))-1)+1);
    float vz = H * (sgn *(2*(1/T-cos(4*PI*t/T)/T)-1)+1);
    float y = targetCoMVelocity[1] * t;
    float vy = targetCoMVelocity[1];
    Matrix<float, 6, 6>jacobian_swingMatrix;
    Vector<float, 6> legPresentVel;
    Vector<float, 6> legCmdVel;
    if (swingFlag == 0)
    {
        legCmdPos(1,0) = x;
        legCmdPos(1,1) = y - 0.8 * legPresentPos(3,1) ;
        legCmdPos(1,2) = -0.24 + z;
        legCmdPos(2,0) = x;
        legCmdPos(2,1) = y - 0.8 * legPresentPos(3,1);
        legCmdPos(2,2) = -0.24 + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                               jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                               jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                               jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                               jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_jointPresentVel;
        temp_jointPresentVel = jointPresentVel.segment(3, 6);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(1,0)- legPresentPos(1,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(1,1)- legPresentPos(1,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(1,2)- legPresentPos(1,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(2,0)- legPresentPos(2,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(2,1)- legPresentPos(2,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(2,2)- legPresentPos(2,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));
        jacobian_torque.segment(3, 6) = jacobian_swingMatrix.transpose() * temp_forceswing;
    }
    else
    {
        legCmdPos(0,0) = x;
        legCmdPos(0,1) = y - 0.8 * legPresentPos(2,1);
        legCmdPos(0,2) = -0.24 + z;
        legCmdPos(3,0) = x;
        legCmdPos(3,1) = y - 0.8 * legPresentPos(2,1);
        legCmdPos(3,2) = -0.24 + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                               jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                               jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                               jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                               jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_jointPresentVel;
        temp_jointPresentVel.head(3) = jointPresentVel.head(3);
        temp_jointPresentVel.tail(3) = jointPresentVel.tail(3);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(0,0)- legPresentPos(0,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(0,1)- legPresentPos(0,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(0,2)- legPresentPos(0,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(3,0)- legPresentPos(3,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(3,1)- legPresentPos(3,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(3,2)- legPresentPos(3,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));
        Vector<float, 6> temp_torqueswing;
        temp_torqueswing = jacobian_swingMatrix.transpose() * temp_forceswing;
        jacobian_torque.head(3) = temp_torqueswing.head(3);
        jacobian_torque.tail(3) = temp_torqueswing.tail(3);
    }
    fly_time += timePeriod;
    Vector<float, 4> temp_3leg1;
    temp_3leg1 << 0, 1, 1, 1;
    Vector<float, 4> temp_3leg2;
    temp_3leg2 << 1, 0, 1, 1;
    Vector<float, 4> temp_3leg3;
    temp_3leg3 << 1, 1, 0, 1;
    Vector<float, 4> temp_3leg4;
    temp_3leg4 << 1, 1, 1, 0;
    if(state == temp_3leg1)
        leg << 1,2,3;
    else if(state == temp_3leg2)
        leg << 0,2,3;
    else if(state == temp_3leg3)
        leg << 0,1,3;
    else if(state == temp_3leg4)
        leg << 0,1,2;
    else
        leg = leg;
    // if (fly_time > T + 0.005)
    // {
    //     fly_time = 0.0;
    //     if (swingFlag == 0)
    //     {
    //         swingFlag = 1;
    //         legPresentPos_target << legPresentPos(2, 0), legPresentPos(2, 1), legPresentPos(2, 2);
    //     }
    //     else
    //     {
    //         swingFlag = 0;
    //         legPresentPos_target << legPresentPos(3, 0), legPresentPos(3, 1), legPresentPos(3, 2);
    //     }
    // }
}
void MotionControl::stance_VMC_3leg()
{
    //建立支撑相集合
    Vector<float, 4> temp_3leg1;
    temp_3leg1 << 0, 1, 1, 1;
    Vector<float, 4> temp_3leg2;
    temp_3leg2 << 1, 0, 1, 1;
    Vector<float, 4> temp_3leg3;
    temp_3leg3 << 1, 1, 0, 1;
    Vector<float, 4> temp_3leg4;
    temp_3leg4 << 1, 1, 1, 0;
    if(state == temp_3leg1)
        leg << 1,2,3;
    else if(state == temp_3leg2)
        leg << 0,2,3;
    else if(state == temp_3leg3)
        leg << 0,1,3;
    else if(state == temp_3leg4)
        leg << 0,1,2;
    else
        leg = leg;
    cout << "leg: " << leg.transpose() << endl;
    target_taoz = target_taoz + targetCoMVelocity[2] * timePeriod;
    float tao_x = k_taox * (0.0 - imu_num[0]) + d_taox * (0 -imuVel(0));
    float tao_y = k_taoy * (0.0 - imu_num[1]) + d_taoy * (0 -imuVel(1));
    float tao_z = k_taoz * (target_taoz - imu_num[2]) + d_taoz * (targetCoMVelocity[2]  -imuVel(2));
    Matrix<float, 6, 9> A;
    Vector<float, 6> B;
    Vector<float, 3> presentCoMVelocity;
    float x1 = leg2CoMPrePos(leg[0], 0);
    float y1 = leg2CoMPrePos(leg[0], 1);
    float z1 = leg2CoMPrePos(leg[0], 2);
    float x2 = leg2CoMPrePos(leg[1], 0);
    float y2 = leg2CoMPrePos(leg[1], 1);
    float z2 = leg2CoMPrePos(leg[1], 2);
    float x3 = leg2CoMPrePos(leg[2], 0);
    float y3 = leg2CoMPrePos(leg[2], 1);
    float z3 = leg2CoMPrePos(leg[2], 2);
    A << 1, 0, 0, 1, 0, 0, 1, 0, 0,
         0, 1, 0, 0, 1, 0, 0, 1, 0,
         0, 0, 1, 0, 0, 1, 0, 0, 1,
         0, -z1, y1, 0, -z2, y2, 0, -z3, y3,
         z1, 0, -x1, z2, 0, -x2, z3, 0, -x3,
         -y1, x1, 0, -y2, x2, 0, -y3, x3, 0;
    Matrix<float, 3, 3>temp_Matrix;
    temp_Matrix << jacobian(leg[2] ,0), jacobian(leg[2] ,1), jacobian(leg[2] ,2),
                   jacobian(leg[2] ,3), jacobian(leg[2] ,4), jacobian(leg[2] ,5),
                   jacobian(leg[2] ,6), jacobian(leg[2] ,7), jacobian(leg[2] ,8);
    Vector<float, 3>temp_vel;
    temp_vel(0) = jointPresentVel(leg[2] * 3 );
    temp_vel(1) = jointPresentVel(leg[2] * 3 + 1);
    temp_vel(2) = jointPresentVel(leg[2] * 3 + 2);
    Vector<float, 3> temp_comvel;
    temp_comvel = -temp_Matrix * temp_vel;
    presentCoMVelocity[0] = temp_comvel(0);
    presentCoMVelocity[1] = temp_comvel(1);
    presentCoMVelocity[2] = temp_comvel(2);
    float Fx = -kx * (0 - legPresentPos(leg[2], 0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
    float Fz = -kz * (-0.24 - legPresentPos(leg[2], 2)) + dz * (0 - presentCoMVelocity(2));
    float Fy = -ky * (0.0009 -legPresentPos(leg[2], 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
    B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    float alpha = 0.001;
    Matrix<float, 9, 9> temp_H;
    Vector<float, 9> temp_g;
    temp_H = 2 * A.transpose() * A + 2 * alpha * MatrixXf::Identity(9, 9);
    temp_g = - 2 * A.transpose() * B;
    Vector<float, 9> temp_Force;
    Matrix<float, 9, 9>jacobian_Matrix;
    Vector<float, 9> temp_torque;
    USING_NAMESPACE_QPOASES
    real_t H[9*9];
    real_t g[9];
    for (int i = 0; i < 9; i++)
    {
        for (int j= 0; j < 9; j++)
        {
            H[9 * i + j] = temp_H(i, j);
        }
        g[i] = temp_g(i);
    }
    float u = 4;
	real_t fA[12*9] = {u, 0, 1, 0, 0, 0, 0, 0, 0,
                       -u, 0, 1, 0, 0, 0, 0, 0, 0,
                       0, u, 1, 0, 0, 0, 0, 0, 0,
                       0, -u, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, u, 0, 1, 0, 0, 0,
                       0, 0, 0, -u, 0, 1, 0, 0, 0,
                       0, 0, 0, 0, u, 1, 0, 0, 0,
                       0, 0, 0, 0, -u, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, u, 0, 1,
                       0, 0, 0, 0, 0, 0, -u, 0, 1,
                       0, 0, 0, 0, 0, 0, 0, u, 1,
                       0, 0, 0, 0, 0, 0, 0, -u, 1};
    real_t lb[9] = {-40, -40, 0, -40, -40, 0, -40, -40, 0};
    real_t ub[9] = {40, 40, 40, 40, 40, 40, 40, 40, 40};
    real_t lbA[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    real_t ubA[12] = {100000, 100000, 1000000, 1000000, 1000000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};
	SQProblem example( 9,12 );
	int_t nWSR = 50;
	example.init( H,g,fA,lb,ub,lbA,ubA,nWSR);
	real_t xOpt[9];
	example.getPrimalSolution( xOpt );
    temp_Force << xOpt[0], xOpt[1], xOpt[2], xOpt[3], xOpt[4], xOpt[5], xOpt[6], xOpt[7], xOpt[8];
    cout << "force:   "<< temp_Force << endl;
    jacobian_Matrix = MatrixXf::Zero(9, 9);
    jacobian_Matrix.block(0,0,3,3) << jacobian(leg[0] ,0), jacobian(leg[0] ,1), jacobian(leg[0] ,2),
                                      jacobian(leg[0] ,3), jacobian(leg[0] ,4), jacobian(leg[0] ,5),
                                      jacobian(leg[0] ,6), jacobian(leg[0] ,7), jacobian(leg[0] ,8);
    jacobian_Matrix.block(3,3,3,3) << jacobian(leg[1] ,0), jacobian(leg[1] ,1), jacobian(leg[1] ,2),
                                      jacobian(leg[1] ,3), jacobian(leg[1] ,4), jacobian(leg[1] ,5),
                                      jacobian(leg[1] ,6), jacobian(leg[1] ,7), jacobian(leg[1] ,8);
    jacobian_Matrix.block(6,6,3,3) << jacobian(leg[2] ,0), jacobian(leg[2] ,1), jacobian(leg[2] ,2),
                                      jacobian(leg[2] ,3), jacobian(leg[2] ,4), jacobian(leg[2] ,5),
                                      jacobian(leg[2] ,6), jacobian(leg[2] ,7), jacobian(leg[2] ,8);
    temp_torque = -jacobian_Matrix.transpose() * temp_Force;
    jacobian_torque.segment(leg[0] * 3, 3) = temp_torque.head(3);
    jacobian_torque.segment(leg[1] * 3, 3) = temp_torque.segment(3,3);
    jacobian_torque.segment(leg[2] * 3, 3) = temp_torque.tail(3);
}
void MotionControl::swing_VMC_3leg()
{
    int fly_leg;//飞行相
    fly_leg = 6 - leg.sum();
    float S = T * targetCoMVelocity[0]/2;
    float t = fly_time ;
    float x = S * (t/T-sin(2*PI*t/T)/(2*PI));
    float vx = S * (1/T-cos(2*PI*t/T)/T);
    float sgn;
    if ( t >= 0 && t < T/2)
    {
       sgn = 1;
    }
    else
    {
       sgn = -1;
    }
    float z = H * (sgn * (2*(t/T-sin(4*PI*t/T)/(4*PI))-1)+1);
    float vz = H * (sgn *(2*(1/T-cos(4*PI*t/T)/T)-1)+1);
    float y = targetCoMVelocity[1] * t;
    float vy = targetCoMVelocity[1];
    Matrix<float, 3, 3>jacobian_swingMatrix;
    Vector<float, 3> legPresentVel;
    Vector<float, 3> legCmdVel;
    legCmdPos(fly_leg, 0) = x;
    legCmdPos(fly_leg, 1) = y - 0.8 * legPresentPos(leg[2],1);
    legCmdPos(fly_leg, 2) = -0.24 + z;
    legCmdVel(0) = vx;
    legCmdVel(1) = vy;
    legCmdVel(2) = vz;
    jacobian_swingMatrix << jacobian(fly_leg, 0), jacobian(fly_leg, 1), jacobian(fly_leg, 2),
                            jacobian(fly_leg, 3), jacobian(fly_leg ,4), jacobian(fly_leg ,5),
                            jacobian(fly_leg ,6), jacobian(fly_leg ,7), jacobian(fly_leg ,8);
    Vector<float, 3>temp_jointPresentVel;
    temp_jointPresentVel = jointPresentVel.segment(fly_leg * 3, 3);
    legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
    Vector<float, 3> temp_forceswing;
    temp_forceswing(0) = swingfx_kp * (legCmdPos(fly_leg,0)- legPresentPos(fly_leg,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
    temp_forceswing(1) = swingfy_kp * (legCmdPos(fly_leg,1)- legPresentPos(fly_leg,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
    temp_forceswing(2) = swingfz_kp * (legCmdPos(fly_leg,2)- legPresentPos(fly_leg,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
    jacobian_torque.segment(fly_leg * 3, 3) = jacobian_swingMatrix * temp_forceswing;
    fly_time += timePeriod;
}
void MotionControl::stance_VMC_4leg()
{
    target_taoz = target_taoz + targetCoMVelocity[2] * timePeriod;
    float tao_x = k_taox * (0.0 - imu_num[0]) + d_taox * (0 -imuVel(0));
    float tao_y = k_taoy * (0.0 - imu_num[1]) + d_taoy * (0 -imuVel(1));
    float tao_z = k_taoz * (target_taoz - imu_num[2]) + d_taoz * (targetCoMVelocity[2]  -imuVel(2));
    Matrix<float, 6, 12> A;
    Vector<float, 6> B;
    Vector<float, 3> presentCoMVelocity;
    float x1 = leg2CoMPrePos(0, 0);
    float y1 = leg2CoMPrePos(0, 1);
    float z1 = leg2CoMPrePos(0, 2);
    float x2 = leg2CoMPrePos(1, 0);
    float y2 = leg2CoMPrePos(1, 1);
    float z2 = leg2CoMPrePos(1, 2);
    float x3 = leg2CoMPrePos(2, 0);
    float y3 = leg2CoMPrePos(2, 1);
    float z3 = leg2CoMPrePos(2, 2);
    float x4 = leg2CoMPrePos(3, 0);
    float y4 = leg2CoMPrePos(3, 1);
    float z4 = leg2CoMPrePos(3, 2);
    A << 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
         0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
         0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
         0, -z1, y1, 0, -z2, y2, 0, -z3, y3, 0, -z4, y4,
         z1, 0, -x1, z2, 0, -x2, z3, 0, -x3, z4, 0, -x4,
         -y1, x1, 0, -y2, x2, 0, -y3, x3, 0, -y4, x4, 0;
    if (swingFlag == 0)
    {
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                       jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                       jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(9);
        temp_vel(1) = jointPresentVel(10);
        temp_vel(2) = jointPresentVel(11);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix*temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);
        float Fx = -kx * (0- legPresentPos(3,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.24 - legPresentPos(3,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * (0.0009  -legPresentPos(3, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    else
    {
        Matrix<float, 3, 3>temp_Matrix;
        temp_Matrix << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                       jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                       jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        Vector<float, 3>temp_vel;
        temp_vel(0) = jointPresentVel(6);
        temp_vel(1) = jointPresentVel(7);
        temp_vel(2) = jointPresentVel(8);
        Vector<float, 3>temp_comvel;
        temp_comvel = -temp_Matrix * temp_vel;
        presentCoMVelocity[0] = temp_comvel(0);
        presentCoMVelocity[1] = temp_comvel(1);
        presentCoMVelocity[2] = temp_comvel(2);
        float Fx = -kx * (0 - legPresentPos(2,0)) + dx * (targetCoMVelocity[0] - presentCoMVelocity[0]);
        float Fz = -kz * (-0.24 - legPresentPos(2,2)) + dz * (0 - presentCoMVelocity(2));
        float Fy = -ky * (0.0009 -legPresentPos(2, 1)) + dy * (targetCoMVelocity[1] - presentCoMVelocity[1]);
        B << Fx - 9.81 * G * sin(-imu_num(1)), Fy, 9.81 * G * cos(-imu_num(1)) + Fz, tao_x, tao_y, tao_z;
    }
    float alpha = 0.001;
    Matrix<float, 12, 12> temp_H;
    Vector<float, 12> temp_g;
    temp_H = 2 * A.transpose() * A + 2 * alpha * MatrixXf::Identity(12, 12);
    temp_g = - 2 * A.transpose() * B;
    Vector<float, 12> temp_Force;
    Matrix<float, 12, 12> jacobian_Matrix;
    USING_NAMESPACE_QPOASES
    real_t H[12*12];
    real_t g[12];
    for (int i = 0; i < 12; i++)
    {
        for (int j= 0; j< 12; j++)
        {
            H[12 * i + j] = temp_H(i, j);
        }
        g[i] = temp_g(i);
    }
    float u = 4;
	real_t fA[16*12] = {u, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       -u, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, u, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, -u, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, u, 0, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, -u, 0, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, u, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, -u, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, u, 0, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, -u, 0, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, u, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, -u, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, u, 0, 1,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, -u, 0, 1,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, u, 1,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -u, 1};
    real_t lb[12] = {-660, -660, 0, -660, -660, 0, -660, -660, 0, -660, -660, 0};
    real_t ub[12] = {660, 660, 660, 660, 660, 660, 660, 660, 660, 660, 660, 660};
    real_t lbA[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    real_t ubA[16] = {100000, 100000, 1000000, 1000000, 1000000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};
	SQProblem example( 12,16 );
	int_t nWSR = 50;
	example.init( H,g,fA,lb,ub,lbA,ubA,nWSR);
	real_t xOpt[12];
	example.getPrimalSolution( xOpt );
    temp_Force << xOpt[0], xOpt[1], xOpt[2], xOpt[3], xOpt[4], xOpt[5], xOpt[6], xOpt[7], xOpt[8], xOpt[9], xOpt[10], xOpt[11];
    cout << "force:   "<< temp_Force.transpose() << endl;
    leg_force = temp_Force;
    jacobian_Matrix = MatrixXf::Zero(12, 12);
    jacobian_Matrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                      jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                      jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
    jacobian_Matrix.block(3,3,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                      jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                      jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
    jacobian_Matrix.block(6,6,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                      jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                      jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
    jacobian_Matrix.block(9,9,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                      jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                      jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
    jacobian_torque = -jacobian_Matrix.transpose() * temp_Force;
}