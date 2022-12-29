%%
%y = ekf(0.1, -6.895369395151447, -6.895369395151447, 4.5, 20.5, 1.1)
clear all;
clc;
%%

mode = 0;
dt = 1;
lat = -6.914564;
lon = 107.609810;
odo_VL = -3;
odo_VR = 4;

%Call Kalman filter to estimate the position
y = ekf(mode,dt,lat,lon,odo_VL,odo_VR)