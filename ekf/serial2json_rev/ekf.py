"""
#!/usr/bin/env python3
#title           :ekf.py
#description     :Python Script for EKF algorithm
#author          :Nicholas Putra Rihandoko
#date            :2022/15/12
#version         :1.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""

## EKF GPS-Tracking
import numpy as np
import pymap3d as pm

## KALMAN FILTER VARIABLES
# Variable initialization
x_est = np.array([[None],[None],[None]])    # estimated states (Px,Py,psi)
p_est = np.eye(3)                           # estimated states' covariance matrix
x_prd = np.array([[None],[None],[None]])    # predicted states (P'x,P'y,psi')
p_prd = np.eye(3)                           # predicted states' covariance matrix
# Calibration Procedure Parameter:
cal = 0             # headings callibration data index 
last = 'point B'    # headings callibration position marker
status = 0          # headings callibration status
# calibration calculation parameter:
px_a = np.zeros((10,1)) # position X at point A
py_a = np.zeros((10,1)) # position y at point A
px_b = np.zeros((10,1)) # position X at point B
py_b = np.zeros((10,1)) # position y at point B
head_a = 0   # headings at point A (uncalibrated)
head_b = 0   # headings at point B (uncalibrated)
# Tuning parameter:
gps_std = 10  # GPS measurments' standard deviation
odo_std = 10  # odometry measurments' standard deviation

# general arc motion variables
s_fwd = None    # forward displacement at local coordinate
s_str = None    # striding displacement at local coordinate

# numerical jacobian perturbation increment
dx = 0.0001
# GPS distance to mid section of tire [m]
L1 = 0
# Tire distance [m]
L2 = 0.325
# GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung)
lla0 = np.array([-6.914744,107.60981,800])

# Tuning Parameter
def setPar(status,x_init,enu):
    global gps_std, odo_std, x_est
    # GPS meas standard deviation [m]
    gps_std = 5
    # odometry meas standard deviation [m/s]
    odo_std = 5
    if status == 1:
        # GPS meas standard deviation [m]
        gps_std = 10
        # odometry meas standard deviation [m/s]
        odo_std = 0.1

    if x_init == None:
        x_est = np.array([[enu[0]],[enu[1]],[0]])

##  ~wrapAngle [to make sure 0 < psi < 2*pi]
def wrapAngle(angle): 
    wrap = angle%(2*np.pi)
    return wrap

## PREDICTION STEP
def predict(x_est,p_est,odo_psi_1dot,dt):
    global odo_std,dx,L2,s_fwd,s_str
    # Predicted state
    x_prd = np.array([[x_est[0,0] + (s_fwd * np.cos(x_est[2,0])) - (s_str * np.sin(x_est[2,0]))],\
        [x_est[1,0] + (s_fwd * np.sin(x_est[2,0])) + (s_str * np.cos(x_est[2,0]))],\
            [wrapAngle(x_est[2,0] + (dt * odo_psi_1dot))]])
    # Jacobian of system function (predicted state)
    jac_fx = np.subtract(np.transpose(np.array([[x_est[0,0]+dx + (s_fwd * np.cos(x_est[2,0])) - (s_str * np.sin(x_est[2,0])),\
        x_est[1,0] + (s_fwd * np.sin(x_est[2,0])) + (s_str * np.cos(x_est[2,0])),\
            wrapAngle(x_est[2,0] + dt * odo_psi_1dot)],\
                [x_est[0,0] + (s_fwd * np.cos(x_est[2,0])) - (s_str * np.sin(x_est[2,0])),\
                    x_est[1,0]+dx + (s_fwd * np.sin(x_est[2,0])) + (s_str * np.cos(x_est[2,0])),\
                        wrapAngle(x_est[2,0] + dt * odo_psi_1dot)],\
                            [x_est[0,0] + (s_fwd * np.cos(x_est[2,0]+dx)) - (s_str * np.sin(x_est[2,0]+dx)),\
                                x_est[1,0] + (s_fwd * np.sin(x_est[2,0]+dx)) + (s_str * np.cos(x_est[2,0]+dx)),\
                                    wrapAngle(x_est[2,0]+dx + dt * odo_psi_1dot)]])),\
                                        np.array([[x_prd[0,0], x_prd[0,0], x_prd[0,0]],\
                                            [x_prd[1,0], x_prd[1,0], x_prd[1,0]],\
                                                [x_prd[2,0], x_prd[2,0], x_prd[2,0]]])) / dx
    # Predicted State Noise Covariance
    Q = [[(odo_std * dt)**2, 0, 0],\
        [0, (odo_std * dt)**2, 0],\
            [0, 0, (odo_std / L2)**2]]
    # Predicted State Total Covariance
    p_prd = np.add(np.matmul(np.matmul(jac_fx, p_est), np.transpose(jac_fx)), np.array(Q))
    return x_prd,p_prd
    
## UPDATE STEP  
def update(x_prd,p_prd,enu):
    global gps_std,dx,L1
    # Measurement Innovation
    z = np.array([[enu[0] + L1*np.cos(x_prd[2,0])], [enu[1] + L1*np.sin(x_prd[2,0])]])
    z_prd = np.array([[x_prd[0,0]], [x_prd[1,0]]])
    u = np.subtract(z,z_prd)
    # Jacobian of measurement function
    jac_hx = np.subtract(np.transpose(np.array([[x_prd[0,0]+dx, x_prd[1,0]],\
        [x_prd[0,0], x_prd[1,0]+dx],\
            [x_prd[0,0], x_prd[1,0]]])),\
                np.array([[z_prd[0,0],z_prd[0,0],z_prd[0,0]],\
                    [z_prd[1,0],z_prd[1,0],z_prd[1,0]]])) / dx
    # Measurement Noise Covariance
    R = [[gps_std**2, 0],\
        [0, gps_std**2]]
    # Kalman Gain
    K = np.matmul(np.matmul(p_prd, np.transpose(jac_hx)), np.linalg.inv(np.add(np.matmul(np.matmul(jac_hx, p_prd), np.transpose(jac_hx)),np.array(R))))
    # Estimated State
    x_est = np.add(x_prd, np.matmul(K,u))
    # Estimated State Covariance
    p_est = np.matmul(np.subtract(np.eye(3,3), np.matmul(K,jac_hx)), p_prd)
    return x_est,p_est

def filtering(mode,dt,lat,lon,odo_VL,odo_VR):
    global dx,L1,L2,lla0,\
        x_est,p_est,x_prd,p_prd,\
            cal,last,status,px_a,py_a,px_b,py_b,head_a,head_b,\
                gps_std,odo_std,s_fwd,s_str

    # GPS coordinate conversion from latitude,longitude [deg] to east-x,north-y [m]
    enu = pm.geodetic2enu(lat,lon,lla0[2],lla0[0],lla0[1],lla0[2])

    # Set standard deviation
    setPar(status,x_est[0,0],enu)

    # odometry measurement
    odo_V = (odo_VL + odo_VR) / 2
    odo_psi_1dot = (odo_VL - odo_VR) / L2
    # arc motion in local coordinate
    if odo_psi_1dot^2 < 0.6^2: # lower threshold for linear motion
        odo_psi_1dot = odo_psi_1dot*0.3
    else:
        if odo_psi_1dot^2 > 2.36^2: # upper threshold for rotation in place
            odo_psi_1dot = odo_psi_1dot*0.3
    # general arc motion
    s_fwd = (odo_V/odo_psi_1dot) * np.sin(dt*odo_psi_1dot)
    s_str = (odo_V/odo_psi_1dot) * (1-np.cos(dt*odo_psi_1dot))
    
    # KALMAN FILTERING
    x_prd,p_prd = predict(x_est,p_est,odo_psi_1dot,dt)
    if mode == 0:
        # without GPS measurement
        x_est = x_prd
        p_est = p_prd
        if last == 'point A':
            head_b = head_b + (dt * odo_psi_1dot)
    else:
        # with GPS measurement
        x_est,p_est = update(x_prd,p_prd,enu)
        # heading callibration
        if mode == 1 and last == 'point A':
            head_b = head_b + (dt * odo_psi_1dot)
        else:
            if mode == 2:
                if last == 'point B':
                    cal = 0
                    last = 'point A'
                    status = 0
                else:
                    if last == 'point A':
                        cal = cal + 1
                px_a[cal,0] = x_est[0,0]
                py_a[cal,0] = x_est[1,0]
                head_a = x_est[2,0]
                head_b = head_a
            else:
                if mode == 3:
                    if last == 'point A':
                        cal = 0
                        last = 'point B'
                    else:
                        if last == 'point B':
                            cal = cal + 1
                    px_b[cal,1] = x_est[0,0]
                    py_b[cal,1] = x_est[1,0]
                    #head_B = x_est(3,1);
                else:
                    if mode == 4:
                        b = (np.mean(py_b) - np.mean(py_a)) / (np.mean(px_b) - np.mean(px_a))
                        if ((np.mean(py_b) - np.mean(py_a)) > 0 and (np.mean(px_b) - np.mean(px_a)) < 0) or ((np.mean(py_b) - np.mean(py_a)) < 0 and (np.mean(px_b) - np.mean(px_a)) < 0):
                            x_est[2,0] = ((head_b - head_a) + 2 * (np.pi + np.arctan(b))) / 2
                        else:
                            x_est[2,0] = ((head_b - head_a) + 2 * np.arctan(b)) / 2
                        status = 1
    
    # Updated Measurements (from estimated state)
    lla = pm.enu2geodetic(x_est[0,0],x_est[1,0],lla0[2],lla0[0],lla0[1],lla0[2])
    result_ekf = np.array([lla[0],lla[1],np.rad2deg(x_est[2,0])])
    return result_ekf