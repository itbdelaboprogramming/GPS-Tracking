#ifndef EKF_H
#define EKF_H

#include <vector>
#include <Eigen/Dense>
//#include <stdio.h>
#include <algorithm>
#include <cmath>

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;

struct VehicleState
{
    double x,y,psi,V;
    double yaw_rate,steering;
    VehicleState():x(0.0), y(0.0), psi(0.0), V(0.0), yaw_rate(0.0), steering(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(0.0),steering(0.0) {}
};

struct GPSMeasurement
{
    double x,y;
    GPSMeasurement(double setX, double setY):x(setX),y(setY) {}
};

struct GyroMeasurement
{
    double psi_dot;
    GyroMeasurement(double setPsi_dot):psi_dot(setPsi_dot) {}
};

struct InputProcess
{
    double psi_dot, V;
    InputProcess(double setPsi_dot, double setV):psi_dot(setPsi_dot), V(setV) {}
};

class ekf
{
    public:

        //ekf():m_initialised(false){}
        ekf(double input[], double millis):L1(10), L2(10){}
        ekf(double gps_read[], double gyro_read, double encod_read[]){} 
        virtual ~ekf(){}
        //void reset(){m_initialised = false;}
        //bool isInitialised() const {return m_initialised;}

        double wrapAngle(double angle);
        void setCons(InputProcess control);

        void predictionStep(InputProcess control, double dt);
        void handleGPSMeasurement(GPSMeasurement meas);
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}
        

    private:
        //bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
        double Cv;
        double alpha;
        double L1;
        double L2;
        double millis;
};

#endif