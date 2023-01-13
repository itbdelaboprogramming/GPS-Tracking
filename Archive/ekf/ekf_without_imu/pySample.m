% Sample script to demonstrate execution of function ekf
mode = true;
dt = 0.1;
lat = -6.895369395151447;
lon = 107.6116112416878;
odo_VL = 20.5;
odo_VR = 20.5;

y = ekf(mode,dt,lat,lon,odo_VL,odo_VR);