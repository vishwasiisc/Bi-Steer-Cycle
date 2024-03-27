%main bisteer 3D

clear;
close all;
clc;

dr = 0.1; df = 0.1;

I11 = 0.1; I22 = 0.1; I33 = 0.1;
  m = 1;   g = 0;    h = 0.1;

%%
%packing parameters
p.dr = dr; p.df = df;

p.I11 = I11; p.I22 = I22; p.I33 = I33;

p.m = m; p.g = g; p.h = h;

%%
% initial condition
x0 = 0;
y0 = 0;
V0  = 0.1;

psi0 = deg2rad(0);  %heading
phi0 = deg2rad(-89);  %lean angle
phidot0 = 0; %lean rate

theta_R0 = deg2rad(0);
theta_F0 = deg2rad(30);   

z0 = [x0,y0,V0,psi0,phi0,phidot0,theta_R0,theta_F0]';

%%
%creat a function
therhs = @(t,z) bisteer_3D_rhs(t,z,p);

%solving parameters
start = 0; stop = 1000; t = linspace(start,stop,1000000);

%solve ode
small   = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small);
soln    = ode45(therhs,t, z0,options);



%%
%plotting
tarray = linspace(start, stop, 100000);

zarray = deval(soln,tarray);

x_array = zarray(1,:);
y_array = zarray(2,:);
V_array = zarray(3,:);
dV_array = V_array-V0;
phi_array = zarray(5,:);
phidot_array = zarray(6,:);


figure(1)
%subplot(1,2,1)
hold on
plot(x_array,y_array);
plot(x_array(1),y_array(1),'b.',MarkerSize=13)
plot(x_array(end),y_array(end),'r.',MarkerSize=13)
legend('','start','end')
title('Trajectory')
xlabel('x')
ylabel('y')
axis equal
movegui('northwest')


figure(2)
%subplot(1,2,2)
plot(tarray,rad2deg(phi_array));
title('lean angle')
xlabel('t');
ylabel('$\phi$ in degrees')
movegui('southwest')

%{
figure(3)
plot(tarray,V_array);
%}

%
figure(3)
%subplot(1,3,3)
plot(tarray,dV_array);
title('velocity veriation')
xlabel('t');
ylabel('$\Delta V$ in degrees')
%}

figure(4)
plot(tarray,phidot_array)
title('lean rate')


%max(phi_array)/pi

%animate_bisteer(soln,stop,2,p)

%%%%%controller%%%%%%%%%









