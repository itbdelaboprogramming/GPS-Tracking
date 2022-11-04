%% EKF GPS-Tracking
%#codegen
function result_ekf = ekf(dt,lat,lon,psi_1dot,V,V_1dot)
%% KALMAN FILTER VARIABLE & TUNING PARAMETER
% dt = time increment [s]
% zx,zy = latitude,longitude GPS measurment [deg]
% psi_1dot = turning velocity [rad/s]
% V = linear velocity [m/s]
% V_1dot = linear acceleration [m/s2]
persistent x_est p_est Cv alpha L1 pos_std psi_1dot_std V_std V_1dot_std dx lla0
                      
% numerical jacobian perturbation increment
dx = 0.001;

% GPS distance to mid section of tire [m]
L1 = 0.356;

% GPS meas standard deviation [m]
pos_std = 10^0/3;

% turning velocity input/meas standard deviation [rad/s]
psi_1dot_std = 3*10^-1;

% linear velocity input/meas standard deviation [m/s2]
V_std = 10^-2;

% linear accel input/meas standard deviation [m/s2]
V_1dot_std = 10^0;

% GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung)
lla0 = [-6.914744, 107.609810, 800];

% GPS coordinate conversion from latitude,longitude [deg] to east-x,north-y [m]
enu = lla2enu([lat,lon,800],lla0,'ellipsoid');

% Initial state & covariance (x_est = [Px; Py; psi; V])
if isempty(x_est)
    x_est = [enu(1,1); enu(1,2); 0; V];     
    p_est = eye(4);
end     

% Skid-steering parameter
if psi_1dot^2 < 0.1
    Cv = 1;
    alpha = 0;
elseif V^2 < 0.3 && psi_1dot > 0
    Cv = L1;
    alpha = pi()/2;
elseif V^2 < 0.3 && psi_1dot < 0
    Cv = L1;
    alpha = -pi()/2;
else
    Cv = sqrt(L1^2 + (V/psi_1dot)^2) / (V/psi_1dot);
    alpha = atan(L1/(V/psi_1dot));
end

%% PREDICTION STEP
% Predicted state
x_prd =[x_est(1, 1) + (dt * Cv * V * sin(x_est(3, 1) - alpha));...            
        x_est(2, 1) + (dt * Cv * V * cos(x_est(3, 1) - alpha));...
        wrapAngle(x_est(3, 1) + (dt * psi_1dot));...
        x_est(4, 1) + (dt * V_1dot)];

% Jacobian of system function (predicted state)
jac_fx = ([x_est(1, 1)+dx + dt * Cv * x_est(4, 1) * sin(x_est(3, 1) - alpha),...
          x_est(2, 1) + dt * Cv * x_est(4, 1) * cos(x_est(3, 1) - alpha),...
          wrapAngle(x_est(3, 1) + dt * psi_1dot),...
          x_est(4, 1) + dt * V_1dot;...
          x_est(1, 1) + dt * Cv * x_est(4, 1) * sin(x_est(3, 1) - alpha),...
          x_est(2, 1)+dx + dt * Cv * x_est(4, 1) * cos(x_est(3, 1) - alpha),...
          wrapAngle(x_est(3, 1) + dt * psi_1dot),...
          x_est(4, 1) + dt * V_1dot;...
          x_est(1, 1) + dt * Cv * x_est(4, 1) * sin(x_est(3, 1)+dx - alpha),...
          x_est(2, 1) + dt * Cv * x_est(4, 1) * cos(x_est(3, 1)+dx - alpha),...
          wrapAngle(x_est(3, 1)+dx + dt * psi_1dot),...
          x_est(4, 1) + dt * V_1dot;...
          x_est(1, 1) + dt * Cv * x_est(4, 1)+dx * sin(x_est(3, 1) - alpha),...
          x_est(2, 1) + dt * Cv * x_est(4, 1)+dx * cos(x_est(3, 1) - alpha),...
          wrapAngle(x_est(3, 1) + dt * psi_1dot),...
          x_est(4, 1)+dx + dt * V_1dot]'...
          - [x_prd x_prd x_prd x_prd]) / dx; 

% Predicted State Noise Covariance
Q = [0 0 0 0;...
    0 0 0 0;...
    0 0 (psi_1dot_std*dt)^2 0;...
    0 0 0 (V_1dot_std*dt)^2];

% Predicted State Total Covariance
p_prd = jac_fx * p_est * jac_fx' + Q;

%% UPDATE STEP
% Measurement Innovation
z = [enu(1,1); enu(1,2); V]; %angleWrap(atan((x_prd(2,1)-x_est(2,1))/(x_prd(1,1)-x_est(1,1))));
z_prd = [x_prd(1,1); x_prd(2,1); x_prd(4,1)];
u = z - z_prd;

% Jacobian of measurement function
jac_hx = ([ x_prd(1,1)+dx, x_prd(2,1), x_prd(4,1);...
            x_prd(1,1), x_prd(2,1)+dx, x_prd(4,1);...
            x_prd(1,1), x_prd(2,1), x_prd(4,1);...
            x_prd(1,1), x_prd(2,1), x_prd(4,1)+dx]'...
            - [z_prd z_prd z_prd z_prd]) / dx; 

% Measurement Noise Covariance
R = [pos_std^2 0 0;...            
    0 pos_std^2 0;...
    0 0 V_std^2];

% Kalman Gain
K = p_prd * jac_hx' / (jac_hx * p_prd * jac_hx' + R);

% Estimated State
x_est = x_prd + K * u;

% Estimated State Covariance
p_est = (eye(size(K*jac_hx)) - K * jac_hx) * p_prd;

% Updated Measurements (from estimated state)
lla = enu2lla([x_est(1,1),x_est(2,1),800],lla0,'ellipsoid');
result_ekf = [lla(1,1), lla(1,2), x_est(3,1), x_est(4,1)];
%rad2deg(x_est(3,1))

%%  ~wrapAngle [to make sure -pi/2 < psi < pi/2]
function wrap = wrapAngle(angle)
    if angle > pi()/2
        wrap = angle - pi() + pi();
    elseif angle < -pi()/2
        wrap = angle + pi() - pi();
    else
        wrap = angle;
    end
end

end