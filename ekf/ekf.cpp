
#include "ekf.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
// -------------------------------------------------- //

ekf::ekf(double input[], double millis)
{
    InputProcess control(input[0], input[1]);

    setCons(control);

    predictionStep(control, millis);
}

void ekf::setCons(InputProcess control)
{
    if (control.psi_dot == 0){
        Cv = 1; alpha = 0;
    }
    else if (control.V == 0 && control.psi_dot > 0){
        Cv = L1; alpha = M_PI;
    }
    else if (control.V == 0 && control.psi_dot < 0){
        Cv = L1; alpha = -M_PI;
    }
    else {
        Cv = sqrt(L1*L1 + (control.V/control.psi_dot))/(control.V/control.psi_dot);
        alpha = atan(L1/(control.V/control.psi_dot));
    }
}

double ekf::wrapAngle(double angle)
{
	angle = fmod(angle, (2.0*M_PI));
	if (angle <= -M_PI){angle += (2.0*M_PI);}
	else if (angle > M_PI){angle -= (2.0*M_PI);}
	return angle;
}

void ekf::predictionStep(InputProcess control, double dt)
{
    VectorXd state = getState();
    MatrixXd cov = getCovariance();

    // Implement The Kalman Filter Prediction Step for the system in the  
    // section below.
    // HINT: Assume the state vector has the form [PX, PY, PSI, V].
    // HINT: Use the Gyroscope measurement as an input into the prediction step.
    // HINT: You can use the constants: ACCEL_STD, GYRO_STD
    // HINT: use the wrapAngle() function on angular values to always keep angle
    // values within correct range, otherwise strange angle effects might be seen.
    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    double x = state(0);
    double y = state(1);
    double psi = state(2);
    double V = state(3);

    // Update State
    double x_new = x + dt * Cv * control.V * cos(psi - alpha);
    double y_new = y + dt * Cv * control.V * sin(psi - alpha);
    double psi_new = wrapAngle(psi + dt * control.psi_dot);
    double V_new = control.V;
    state << x_new,y_new,psi_new,V_new;

    // Generate F Matrix
    MatrixXd F = Matrix4d::Zero();
    F << 1,0,-dt*V*sin(psi),dt*cos(psi),0,1,dt*V*cos(psi),dt*sin(psi),0,0,1,0,0,0,0,1;

    // Generate Q Matrix
    MatrixXd Q = Matrix4d::Zero();
    Q(2,2) = dt*dt*GYRO_STD*GYRO_STD;
    Q(3,3) = dt*dt*ACCEL_STD*ACCEL_STD;

    cov = F * cov * F.transpose() + Q;

    // ----------------------------------------------------------------------- //

    setState(state);
    setCovariance(cov);
}

void ekf::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the UKF update state would just produce the same result.
    VectorXd state = getState();
    MatrixXd cov = getCovariance();

    VectorXd z = Vector2d::Zero();
    MatrixXd H = MatrixXd(2,4);
    MatrixXd R = Matrix2d::Zero();

    z << meas.x,meas.y;
    H << 1,0,0,0,0,1,0,0;
    R(0,0) = GPS_POS_STD*GPS_POS_STD;
    R(1,1) = GPS_POS_STD*GPS_POS_STD;

    VectorXd z_hat = H * state;
    VectorXd y = z - z_hat;
    MatrixXd S = H * cov * H.transpose() + R;
    MatrixXd K = cov*H.transpose()*S.inverse();

    state = state + K*y;
    cov = (Matrix4d::Identity() - K*H) * cov;

    setState(state);
    setCovariance(cov);
}