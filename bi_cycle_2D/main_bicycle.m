clear;
close all;
clc;


%bi-steer parameters
dr = 0.1; df = 0.1; m = 1; I33 = 0.1;

%packing parameters
p.dr = dr;
p.df = df;
p.m = m; p.I33 = I33;

x0 = 0;
y0 = 0;
V0 = 0.1;

psi0     = 10;
theta0 = deg2rad(30);

%psidot0 = (V0*sin(theta0))/(cos(theta0)*(df + dr));

z0 = [x0, y0, V0, psi0, theta0]';

%create function
therhs = @(t,z) bicycle_rhs(t,z,p);

%solving parameters
start = 0; stop =500; t = linspace(start,stop,1000000);

%solve ode
small   = 1e-6;
options = odeset('AbsTol', small, 'RelTol', small); %AbsTol
soln    = ode45(therhs,t, z0,options);



tarray = linspace(start, stop, 100000);

zarray = deval(soln,tarray);

x_array = zarray(1,:);
y_array = zarray(2,:);
V_array = zarray(3,:);

figure(1)
hold on
plot(x_array,y_array);
plot(x_array(1),y_array(1),'b.',MarkerSize=13)
plot(x_array(end),y_array(end),'r.',MarkerSize=13)
legend('','start','end')
title(' Trajectory')
xlabel('x')
ylabel('y')
axis equal
movegui('northwest')


figure(2)
plot(tarray,V_array-V0);
movegui('south')

