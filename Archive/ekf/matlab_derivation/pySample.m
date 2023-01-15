% Sample script to demonstrate execution of function ekf
mode = true;
dt = 0.1;
lat = -6.895369395151447;
lon = 107.6116112416878;
psi_1dot = 4.5;
V = 20.5;
V_1dot = 1.1;
psi0 = 31.4;

y = ekf(mode,dt,lat,lon,psi_1dot,V,V_1dot,psi0);