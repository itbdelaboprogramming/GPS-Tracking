% EKF GPS-Tracking
%#codegen
function P = ekf(dt,lat,lon,psi_1dot,V,V_1dot)
%Kalman Filter input
% dt = time increment [s]
% zx,zy = latitude,longitude GPS measurment [deg]
% psi_1dot = turning velocity [rad/s]
% V = linear velocity [m/s]
% V_1dot = linear acceleration [m/s2]

%Kalman Filter Tuning Parameter
dx = 0.01;                 % numerical jacobian perturbation increment
L1 = 0.3;                  % GPS distance to mid section of tire [m]
pos_std = 10^0/3;        % GPS meas standard deviation [m]
psi_1dot_std = 10^1;      % turning velocity input/meas standard deviation [rad/s]
v_1dot_std = 10^-2;        % linear accel input/meas standard deviation [m/s2]

%GPS-coordinate conversion
lla0 = [-6.914744, 107.609810, 800];            % latitude [deg] longitude [deg] altitude [m] Bandung
enu = lla2enu([lat,lon,800],lla0,'ellipsoid');  % change position from [deg] to [m]
z = [enu(1,1); enu(1,2)];                       % east-x [m], north-y [m]

% Initial state conditions
persistent x_est p_est Cv alpha         
if isempty(x_est)
    x_est = [z(1); z(2); 0; V];     % x_est=[Px,Py,psi,V]'
    p_est = eye(4);
end     

% Skid-steering parameter
if psi_1dot == 0
    Cv = 1;
    alpha = 0;
elseif V == 0 && psi_1dot > 0
    Cv = L1;
    alpha = pi()/2;
elseif V == 0 && psi_1dot < 0
    Cv = L1;
    alpha = -pi()/2;
else
    Cv = sqrt(L1^2 + (V/psi_1dot)^2) / (V/psi_1dot);
    alpha = atan(L1/(V/psi_1dot));
end

% Predicted state and covariance
x_prd =[x_est(1, 1) + (dt * Cv * x_est(4, 1) * sin(x_est(3, 1) - alpha));...    % [Px]            
        x_est(2, 1) + (dt * Cv * x_est(4, 1) * cos(x_est(3, 1) - alpha));...    % [Py]
        wrapAngle(x_est(3, 1) + (dt * psi_1dot));...                            % [psi]
        x_est(4, 1) + (dt * V_1dot)];                                           % [V]

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
          x_est(4, 1)+dx + dt * V_1dot]' - [x_prd x_prd x_prd x_prd]) / dx;

Q = [0 0 0 0;...
    0 0 0 0;...
    0 0 (psi_1dot_std*dt)^2 0;...
    0 0 0 (v_1dot_std*dt)^2];

p_prd = jac_fx * p_est * jac_fx' + Q;

% Estimation
u = z - x_prd(1:2,1);           % measurement innovation

jac_hx = ([ x_prd(1,1)+dx, x_prd(2,1);...
            x_prd(1,1), x_prd(2,1)+dx;...
            x_prd(1,1), x_prd(2,1);...
            x_prd(1,1), x_prd(2,1)]' - [x_prd(1:2,1) x_prd(1:2,1) x_prd(1:2,1) x_prd(1:2,1)]) / dx;

R = [pos_std^2 0;...            % measurement covariance
    0 pos_std^2];

S = jac_hx * p_prd * jac_hx' + R;
klm_gain = p_prd * jac_hx' / S;

% Estimated state and covariance
x_est = x_prd + klm_gain * u;
p_est = (eye(size(klm_gain*jac_hx)) - klm_gain * jac_hx) * p_prd;

% Compute the estimated measurements
lla = enu2lla([x_est(1,1),x_est(2,1),800],lla0,'ellipsoid');
P = [lla(1,1), lla(1,2), x_est(3,1), x_est(4,1)];

% wrapAngle to make sure -pi/2 < psi < pi/2
function wrap = wrapAngle(angle)
    if angle >= pi()/2
        wrap = angle - pi();
    elseif angle < pi()/2
        wrap = angle + pi();
    else
        wrap = angle;
    end
end

end     % of the function