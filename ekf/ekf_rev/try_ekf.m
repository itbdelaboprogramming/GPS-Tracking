%%
%y = ekf(1, 0.1, -6.895369395151447, 107.6116112416878, 4.5, 20.5, 1.1)
clear all; clc;
%%
% datum [-6.914744, 107.609810, 800] 

dt = 0.1;         % 0.00001 degree [lla] ~~ 1 meter [enu]
numPts = 135;
t = dt:dt:dt*20;
tplot = 1:1:numPts;

% start test
lat01 = -6.914764*ones(1,15);
lon01 = 107.609810*ones(1,15);

% linear test
lat02 = -6.914754:0.00001:-6.914564;
lon02 = 107.609810*ones(1,20);

% turning (rotation only) test
lat03 = -6.914564*ones(1,15);
lon03 = 107.609810*ones(1,15);

% curving test
lat04 = cell2mat(struct2cell(load('lat04.mat')));
lon04 = cell2mat(struct2cell(load('lon04.mat')));

% turning (rotation only) test;
lat05 = lat04(1,end)*ones(1,15);
lon05 = lon04(1,end)*ones(1,15);

% linear test
lat06 = lat05(1,end)*ones(1,15);
lon06 = lon05(1,end):0.00001:(lon05(1,end)+0.00001*15-0.000005);

% stop test
lat07 = lat06(1,end)*ones(1,10);
lon07 = lon06(1,end)*ones(1,10);

% combine all test into one path
lat0=[lat01,lat02,lat03,lat03,lat04,lat05,lat06,lat07];
lon0=[lon01,lon02,lon03,lon03,lon04,lon05,lon06,lon07];

figure;hold;grid;            % Prepare plot window
%result = zeros(100,4);

%GPS-coordinate conversion
lla0 = [-6.914744, 107.609810, 800];            % latitude [deg] longitude [deg] altitude [m] Bandung
err_GPS = zeros(1,135);
err_EKF = zeros(1,135);

% Main loop
for idx = 1:numPts
    mode = 1;
    % noise/distrubance setings
    % GPS
    noise_lat = (rand()-0.5)*2/10^5 * 2;     
    noise_lon = (rand()-0.5)*2/10^5 * 2;
    % odometry
    noise_VL = (rand()-0.5)*2 * 0.1;
    noise_VR = (rand()-0.5)*2 * 0.1;

    lat = lat0(idx) + noise_lat;
    lon = lon0(idx) + noise_lon;
    
    % Input data for each test section
    if idx < 15
        V = 0; w = 0; V_1dot=0; mode=2;
    elseif idx == 15
        V_1dot=11/dt;
    elseif idx > 15 && idx < 35
        V = 11; w = 0; V_1dot=0;
    elseif idx == 35
        V_1dot=-11/dt;
    elseif idx > 35 && idx < 50
        V = 0; w = 0;  V_1dot=0; mode=3;
    elseif idx == 50
        V = 0; w = 0;  V_1dot=0; mode=4;
    elseif idx > 50 && idx < 65  
        V = 0; w = -1.005;  V_1dot=0;
    elseif idx == 65
        V_1dot=9.79/dt;
    elseif idx > 65 && idx < 100
        V = 9.79; w = -0.895; V_1dot=0;
    elseif idx == 100
        V_1dot=-9.79/dt;
    elseif idx > 100 && idx < 115
        V = 0; w = 1.925;  V_1dot=0;
    elseif idx == 116
        V_1dot=11.00/dt;
    elseif idx > 116 && idx < 130
        V = 11.054; w = 0;  V_1dot=0;
    elseif idx == 130
        V_1dot=-11.03/dt;
    elseif idx > 130
        V = 0; w = 0;  V_1dot=0;
    end

    % mode = 1 --> prediction + update (Closeloop) --> default
    % mode = 0 --> prediction only (no GPS measurement / Openloop)
    if rem(idx,3) ~= 0 && mode ~= 4
        mode = 0;
    end
    
    % convert V & w to VL & VR
    odo_VL = (2*V - 0.325*w)/2;
    odo_VR = (2*V + 0.325*w)/2;
    %w = (odo_VR-odo_VL)/0.325;
    %v = (odo_VR+odo_VL)/2;

    % Call Kalman filter to estimate the position    
    y = ekf(mode,dt,lat,lon,odo_VL+noise_VL,odo_VR+noise_VR);
    % change the latitude,longitude to x,y in meters
    z0_enu = lla2enu([lat0(idx),lon0(idx),800],lla0,'ellipsoid');
    z_enu = lla2enu([lat,lon,800],lla0,'ellipsoid');
    y_enu = lla2enu([y(1),y(2),800],lla0,'ellipsoid');
    % Plot the result
    plot_trajectory(mode,...
        [z_enu(1),z_enu(2)],...
        [y_enu(1),y_enu(2)],...
        [z0_enu(1),z0_enu(2)]);
    pause(0.005);
    err_GPS(1,idx) = pos_err(z_enu(1),z_enu(2),z0_enu(1),z0_enu(2));
    err_EKF(1,idx) = pos_err(y_enu(1),y_enu(2),z0_enu(1),z0_enu(2));
    track(idx,1:2) = [z_enu(1),z_enu(2)];
end
plot_error(tplot,err_GPS,err_EKF)

%%
function plot_error(tplot,err_GPS,err_EKF)
figure(2)
plot(tplot,err_GPS,'r',tplot,err_EKF,'g');
title('Error of GPS [red] and its EKF estimate [green]');
xlabel('data');
ylabel('position error [m]');
%axis([0, 100, 0, 3]);
grid on
hold on
disp("GPS data freq. is 0.5x of odometry")
disp("(GPS data is slower or less available)")
disp("___")
disp('max & mean error of 1st-140th data')
disp('of GS measurement only (without EKF)')
error_gps = [max(err_GPS),mean(err_GPS)]
disp('max & mean error of 1st-140th data')
disp('before EKF heading callibration')
error_ekf = [max(err_EKF(1:50)),mean(err_EKF(1:50))]
disp('max & mean error of 51th-140th data')
disp('after EKF heading callibration')
error_ekf_call = [max(err_EKF(51:end)),mean(err_EKF(51:end))]
%hold;
end

function error = pos_err(x,y,zx,zy)
    error = sqrt((x-zx)^2+(y-zy)^2);
end

function plot_trajectory(mode,z,y,z0)
figure(1)
title('GPS measurement [red] its EKF estimate [green]');
xlabel('horizontal position [m]');
ylabel('vertical position [m]');
plot(z0(1), z0(2), 'bx-');
if mode > 0
    plot(z(1), z(2), 'rx-');
end
plot(y(1), y(2), 'go-');
axis([-6, 23, -8, 25]);
hold on
end            % of the function
