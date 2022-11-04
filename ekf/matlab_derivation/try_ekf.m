%%
%y = ekf(0.1, -6.895369395151447, 107.6116112416878, 4.5, 20.5, 1.1)

%%
% datum [-6.914744, 107.609810, 800] 

dt = 0.1;         % 0.00001 degree [lla] ~~ 1 meter [enu]
numPts = 150;
t = dt:dt:dt*20;
tcurve = dt:dt:dt*100;

% stop test
lat01 = -6.914754*ones(1,30);
lon01 = 107.609810*ones(1,30);

% linear test [psi_1dot = 0 rad/s | V = 10.5 m/s]
lat02 = -6.914744:0.00001:-6.914554;
lon02 = 107.609810 + 0*0.00005*sin(2*pi()*0.1*t);

% curve test [psi_1dot = 0.6rad/s | V = 13.5 m/s]
lat03 = -6.914554 + 0.0001 * cos(2*pi()*0.05*tcurve) - 0.0001;
lon03 = 107.609810 + 0.0001 * sin(2*pi()*0.05*tcurve);

V_1dot = 0;
lat0=[lat01,lat02,lat03];
lon0=[lon01,lon02,lon03];

figure;hold;grid;            % Prepare plot window
%result = zeros(100,4);

%GPS-coordinate conversion
lla0 = [-6.914744, 107.609810, 800];            % latitude [deg] longitude [deg] altitude [m] Bandung
%z_enu = zeros(3,100);
%y_enu = zeros(3,100);

% Main loop
for idx = 1:numPts
    
    noise_lat = (rand()-0.5)*2/10^5 * 1;     
    noise_lon = (rand()-0.5)*2/10^5 * 1;
    lat = lat0(idx) + noise_lat;
    lon = lon0(idx) + noise_lon;
    
    if idx <= 30
        V = 0; psi_1dot = 0;
    elseif idx > 30 && idx <= 50
        V = 10.5; psi_1dot = 0;
    elseif idx > 50
        V = 13.5; psi_1dot = 0.6;
    end

    y = ekf(dt,lat,lon,psi_1dot,V,V_1dot);        % Call Kalman filter to estimate the position

    %result(idx,:) = y;
    z0_enu = lla2enu([lat0(idx),lon0(idx),800],lla0,'ellipsoid');
    z_enu = lla2enu([lat,lon,800],lla0,'ellipsoid');
    y_enu = lla2enu([y(1),y(2),800],lla0,'ellipsoid');
    plot_trajectory([z_enu(1),z_enu(2)],[y_enu(1),y_enu(2)],[z0_enu(1),z0_enu(2)]);    % Plot the results
    axis([-6, 16, -5, 25]);
    pause(0.02);
end
hold;

function plot_trajectory(z,y,x)
title('Trajectory of object [blue] its Kalman estimate [green]');
xlabel('horizontal position');
ylabel('vertical position');
plot(x(1), x(2), 'bx-');
plot(z(1), z(2), 'rx-');
plot(y(1), y(2), 'go-');
end            % of the function
