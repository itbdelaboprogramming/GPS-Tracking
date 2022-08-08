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

double wrapAngle(double angle);

struct VehicleState
{
    double x,y,psi,V;
    double yaw_rate,steering;
    VehicleState():x(0.0), y(0.0), psi(0.0), V(0.0), yaw_rate(0.0), steering(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(0.0),steering(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV, double setPsiDot, double setSteering):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(setPsiDot),steering(setSteering) {}
};

class ekf
{
    public:

        ekf():m_initialised(false){}
        virtual ~ekf(){}
        void reset(){m_initialised = false;}
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
};

class KalmanFilter : public ekf
{
    public:
        double wrapAngle(double angle);

        VehicleState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void handleGPSMeasurement(GPSMeasurement meas);

};

#endif