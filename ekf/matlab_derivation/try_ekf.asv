%%
%y = ekf(0.1, -6.895369395151447, 107.6116112416878, 4.5, 20.5, 1.1)
clear all
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

% turn test
lat03 = -6.914564*ones(1,15);
lon03 = 107.609810*ones(1,15);

% curve test
lat04 = cell2mat(struct2cell(load('lat04.mat')));
lon04 = cell2mat(struct2cell(load('lon04.mat')));

% turn test;
lat05 = lat04(1,end)*ones(1,15);
lon05 = lon04(1,end)*ones(1,15);

% linear test
lat06 = lat05(1,end)*ones(1,15);
lon06 = lon05(1,end):0.00001:(lon05(1,end)+0.00001*15-0.000005);

% stop test
lat07 = lat06(1,end)*ones(1,10);
lon07 = lon06(1,end)*ones(1,10);

lat0=[lat01,lat02,lat03,lat03,lat04,lat05,lat06,lat07];
lon0=[lon01,lon02,lon03,lon03,lon04,lon05,lon06,lon07];

figure;hold;grid;            % Prepare plot window
%result = zeros(100,4);

%GPS-coordinate conversion
lla0 = [-6.914744, 107.609810, 800];            % latitude [deg] longitude [deg] altitude [m] Bandung
err_GPS = zeros(1,135);
err_EKF = zeros(1,135);

% Main loop
for idx = 1:135
    
    noise_lat = (rand()-0.5)*2/10^5 * 2.7;     
    noise_lon = (rand()-0.5)*2/10^5 * 2.7;
    noise_V = (rand()-0.5)*2/10^5 * 0.5;
    noise_acc = (rand()-0.5)*2/10^5 * 0.1;
    noise_gyro = (rand()-0.5)*2/10^5 * 0.05;
    lat = lat0(idx) + noise_lat;
    lon = lon0(idx) + noise_lon;
    
    % Input data for each test section
    if idx < 15
        V = 0; psi_1dot = 0; V_1dot=0; mode=2;
    elseif idx == 15
        V_1dot=11/dt; mode=1;
    elseif idx > 15 && idx < 35
        V = 11; psi_1dot = 0; V_1dot=0;
    elseif idx == 35
        V_1dot=-11/dt;
    elseif idx > 35 && idx < 50
        V = 0; psi_1dot = 0;  V_1dot=0; mode=3;
    elseif idx == 50
        V = 0; psi_1dot = 0;  V_1dot=0; mode=4;
    elseif idx > 50 && idx < 65  
        V = 0; psi_1dot = 1.05;  V_1dot=0; mode=1;
    elseif idx == 65
        V_1dot=9.8/dt;
    elseif idx > 65 && idx < 100
        V = 9.8; psi_1dot = 0.90; V_1dot=0;
    elseif idx == 100
        V_1dot=-9.8/dt;
    elseif idx > 100 && idx < 115
        V = 0; psi_1dot = -1.97;  V_1dot=0;
    elseif idx == 116
        V_1dot=11.05/dt;
    elseif idx > 116 && idx < 130
        V = 11.05; psi_1dot = 0;  V_1dot=0;
    elseif idx == 130
        V_1dot=-11.05/dt;
    elseif idx > 130
        V = 0; psi_1dot = 0;  V_1dot=0;
    end

    %
    % true = prediksi + update (Closeloop)
    % false  = hanya prediksi (tidak diupdate pengukuran GPS / Openloop)
    if rem(idx,1) ~= 0 
        mode = 0;
    end
    %}    

    % Call Kalman filter to estimate the position    
    y = ekf(mode,dt,lat,lon,psi_1dot+noise_gyro,V+noise_V,V_1dot+noise_acc);
    %y(3)
    z0_enu = lla2enu([lat0(idx),lon0(idx),800],lla0,'ellipsoid');
    z_enu = lla2enu([lat,lon,800],lla0,'ellipsoid');
    y_enu = lla2enu([y(1),y(2),800],lla0,'ellipsoid');
    % Plot the result
    plot_trajectory(mode,...
        [z_enu(1),z_enu(2)],...
        [y_enu(1),y_enu(2)],...
        [z0_enu(1),z0_enu(2)]);
    pause(0.01);
    err_GPS(1,idx) = pos_err(z_enu(1),z_enu(2),z0_enu(1),z0_enu(2));
    err_EKF(1,idx) = pos_err(y_enu(1),y_enu(2),z0_enu(1),z0_enu(2));
    track(idx,1:2) = [z_enu(1),z_enu(2)];
end
figure(2)
title('Error of GPS [red] and its EKF estimate [green]');
xlabel('data');
ylabel('position error [m]');
plot(tplot,err_GPS,'r',tplot,err_EKF,'g');
%axis([0, 100, 0, 3]);
grid on
hold on
error_gps = [max(err_GPS),mean(err_GPS)]
error_ekf = [max(err_EKF(1:50)),mean(err_EKF(1:50))]
error_ekf_call = [max(err_EKF(51:end)),mean(err_EKF(51:end))]
%hold;

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
