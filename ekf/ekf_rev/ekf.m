%% EKF GPS-Tracking
%#codegen

% mode = toggle between procedures in EKF algorithm
% dt = time increment [s]
% lat,lon = latitude,longitude GPS measurment [deg]
% odo_VL,odo_VR = left & right velocity from odometry measurement
function result_ekf = ekf(mode,dt,lat,lon,odo_VL,odo_VR)
%% KALMAN FILTER VARIABLES
% Variable initialization
persistent x_est p_est ... % estimated states and their covariance matrix
    L1 L2 s_fwd s_str ... % system's geometric parameter
    dx lla0 ... % calculation parameter
    cal last status ... % callibration procedure parameter
    px_a py_a px_b py_b head_a head_b ... % callibration calculation parameter
    gps_std odo_std ... % tuning parameter
% numerical jacobian perturbation increment
dx = 0.0001;
% GPS distance to mid section of wheel [m]
L1 = 0;
% Wheel distance [m]
L2 = 0.325;
% GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung)
lla0 = [-6.914744, 107.609810, 800];

% CONVERT VARIABLE (GPS)
% coordinate conversion from latitude,longitude [deg] to east-x,north-y [m]
enu = lla2enu([lat,lon,800],lla0,'ellipsoid');

% Initial state & covariance
if isempty(x_est)
    x_est = [enu(1,1); enu(1,2); -pi()/2];     
    p_est = eye(3);
    cal = 0; last = "point B"; status = 0;
end
if isempty(px_a)
    px_a = zeros(10,1);
    py_a = zeros(10,1);
end
if isempty(px_b)
    px_b = zeros(10,1);
    py_b = zeros(10,1);
end
if isempty(head_a)
    head_a = 0;
    head_b = 0;
end
% 1st SET of standard deviation (used before & during heading calibration)
% GPS meas standard deviation [m]
gps_std = 4;
% odometry linear velocity meas standard deviation [m/s]
odo_std = 5;
% 2nd SET of standard deviation (used after heading calibration)
if status == 1
    % GPS meas standard deviation [m]
    gps_std = 3;
    % odometry linear velocity meas standard deviation [m/s]
    odo_std = 0.1;
end

% CONVERT VARIABLE (odometry)
% odo_V = forwart velocity [m/s]
odo_V = (odo_VR + odo_VL)/2;
% odo_w = turning velocity [rzad/s]
odo_w = (odo_VR - odo_VL)/L2;

% arc motion in local coordinate
if odo_w == 0
    s_fwd = odo_V*dt;
    s_str = 0;
else
    if odo_w^2 < 0.6^2 % lower threshold for linear motion
        odo_w = odo_w*0.3;
    elseif odo_w^2 > 2.36^2 % upper threshold for rotation in place
        odo_w = odo_w*0.3;
    end
    r = odo_V/odo_w;
    s_fwd = r * sin(dt*odo_w);
    s_str = r * (1-cos(dt*odo_w));
end

% KALMAN FILTERING
[x_prd,p_prd] = predict(x_est,p_est);
if mode == 0
    % without GPS measurement
    x_est = x_prd;
    p_est = p_prd;
    if last == "point A"
        head_b = head_b + (dt * odo_w);
    end
else
    % with GPS measurement
    [x_est,p_est] = update(x_prd,p_prd,enu);
    % heading callibration
    if mode == 1 && last == "point A"
        head_b = head_b + (dt * odo_w);
    elseif mode == 2
        if last == "point B"
            cal = 1; last = "point A"; status = 0;
        elseif last == "point A"
            cal = cal + 1;
        end
        px_a(cal,1) = x_est(1,1);
        py_a(cal,1) = x_est(2,1);
        head_a = x_est(3,1);
        head_b = head_a;
    elseif mode == 3
        if last == "point A"
            cal = 1; last = "point B";
        elseif last == "point B"
            cal = cal + 1;
        end
        px_b(cal,1) = x_est(1,1);
        py_b(cal,1) = x_est(2,1);
    elseif mode == 4
        m = ((mean(py_b)-mean(py_a))/mean(px_b)-mean(px_a));
        if ((mean(py_b)-mean(py_a)) > 0 && (mean(px_b)-mean(px_a)) < 0) ...
                || ((mean(py_b)-mean(py_a)) < 0 && (mean(px_b)-mean(px_a)) > 0)
            x_est(3,1) = ((head_b-head_a) + 2*(pi() + atan(m)))/2;
        else
            x_est(3,1) = ((head_b-head_a) + 2*atan(m))/2;
        end
        status = 1;
    end
end
    % Updated Measurements (from estimated state)
    lla = enu2lla([x_est(1,1),x_est(2,1),800],lla0,'ellipsoid');
    result_ekf = [lla(1,1), lla(1,2), rad2deg(x_est(3,1))];

%% PREDICTION STEP
function [x_prd,p_prd] = predict(x_est,p_est)
%{
% LINEAR KINEMATICS
% Predicted state
x_prd =[x_est(1, 1) + dt * odo_V * cos(x_est(3, 1) + (dt * odo_w)/2);...            
        x_est(2, 1) + dt * odo_V * sin(x_est(3, 1) + (dt * odo_w)/2);...
        wrapAngle(x_est(3, 1) + (dt * odo_w))];
% Jacobian of system function (predicted state)
jac_fx = ([x_est(1, 1)+dx + dt * odo_V * cos(x_est(3, 1) + (dt * odo_w)/2),...
          x_est(2, 1) + dt * odo_V * sin(x_est(3, 1) + (dt * odo_w)/2),...
          wrapAngle(x_est(3, 1) + dt * odo_w);...
          x_est(1, 1) + dt * odo_V * cos(x_est(3, 1) + (dt * odo_w)/2),...
          x_est(2, 1)+dx + dt * odo_V * sin(x_est(3, 1) + (dt * odo_w)/2),...
          wrapAngle(x_est(3, 1) + dt * odo_w);...
          x_est(1, 1) + dt * odo_V * cos(x_est(3, 1)+dx + (dt * odo_w)/2),...
          x_est(2, 1) + dt * odo_V * sin(x_est(3, 1)+dx + (dt * odo_w)/2),...
          wrapAngle(x_est(3, 1)+dx + dt * odo_w)]'...
          - [x_prd x_prd x_prd]) / dx;
%}
% ARC MOTION KINEMATICS
% Predicted state
x_prd =[x_est(1, 1) + s_fwd * cos(x_est(3, 1)) - s_str * sin(x_est(3, 1)) ;...            
        x_est(2, 1) + s_fwd * sin(x_est(3, 1)) + s_str * cos(x_est(3, 1)) ;...
        wrapAngle(x_est(3, 1) + (dt * odo_w))];
% Jacobian of system function (predicted state)
jac_fx = ([x_est(1, 1)+dx + s_fwd * cos(x_est(3, 1)) - s_str * sin(x_est(3, 1)) ,...
          x_est(2, 1) + s_fwd * sin(x_est(3, 1)) + s_str * cos(x_est(3, 1)) ,...
          wrapAngle(x_est(3, 1) + dt * odo_w);...
          x_est(1, 1) + s_fwd * cos(x_est(3, 1)) - s_str * sin(x_est(3, 1)) ,...
          x_est(2, 1)+dx + s_fwd * sin(x_est(3, 1)) + s_str * cos(x_est(3, 1)) ,...
          wrapAngle(x_est(3, 1) + dt * odo_w);...
          x_est(1, 1) + s_fwd * cos(x_est(3, 1)+dx) - s_str * sin(x_est(3, 1)+dx) ,...
          x_est(2, 1) + s_fwd * sin(x_est(3, 1)+dx) + s_str * cos(x_est(3, 1)+dx) ,...
          wrapAngle(x_est(3, 1)+dx + dt * odo_w)]'...
          - [x_prd x_prd x_prd]) / dx;
%}
% Predicted State Noise Covariance
Q = [(odo_std*dt)^2 0 0;...
    0 (odo_std*dt)^2 0;...
    0 0 (odo_std/L2)^2];

% Predicted State Total Covariance
p_prd = jac_fx * p_est * jac_fx' + Q;
end

%% UPDATE STEP
function [x_est,p_est] = update(x_prd,p_prd,enu)
% Measurement Innovation
z = [enu(1,1) + L1*cos(x_prd(3,1)); enu(1,2) + L1*sin(x_prd(3,1))];
z_prd = [x_prd(1,1); x_prd(2,1)];
u = z - z_prd;

% Jacobian of measurement function
jac_hx = ([ x_prd(1,1)+dx, x_prd(2,1);...
            x_prd(1,1), x_prd(2,1)+dx;...
            x_prd(1,1), x_prd(2,1)]'...
            - [z_prd z_prd z_prd]) / dx; 

% Measurement Noise Covariance
R = [gps_std^2 0;...            
    0 gps_std^2];

% Kalman Gain
K = p_prd * jac_hx' / (jac_hx * p_prd * jac_hx' + R);

% Estimated State
x_est = x_prd + K * u;

% Estimated State Covariance
p_est = (eye(size(K*jac_hx)) - K * jac_hx) * p_prd;
end

%%  ~wrapAngle [to make sure -pi/2 < theta < pi/2]
function wrap = wrapAngle(angle)
    wrap = mod(angle,2*pi());
end

end