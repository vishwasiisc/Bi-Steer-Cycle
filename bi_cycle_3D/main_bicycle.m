clear;
close all;
clc;

%%
%cycle parameters
dr = 0.1;  df = 0.1; h=0.1; m = 1; 
I11= 0.1; I22 = 0.1;  I33 = 0.1; 

g = 10;
%%
%packing parameters
p.dr = dr;
p.df = df; p.g = g; p.h = h;
p.m = m; p.I33 = I33; p.I22 = I22; p.I11 = I11;

%%
%initial conditions
x0 = 0;
y0 = 0;
V0 = 1;


phi0    = deg2rad(10);
phidot0 = deg2rad(0);

theta0 =  deg2rad(0);

psi0    = deg2rad(0);

z0 = [x0, y0, V0, psi0, phi0, phidot0, theta0]';


%%
%%%%%%%energy check%%%%%%%%%

psidot0 = (V0*sin(theta0))/(cos(theta0)*(df + dr));

E0 = m*g*h*cos(phi0)...
    +0.5*m*(V0^2 + dr^2*psidot0^2 + h^2*phidot0^2 + h^2*psidot0^2 - h^2*psidot0^2*cos(phi0)^2 + 2*V0*h*psidot0*sin(phi0) - 2*dr*h*phidot0*psidot0*cos(phi0))...
    +0.5*I11*phidot0*phidot0...
    +0.5*I22*psidot0*psidot0*sin(phi0)*sin(phi0)...
    +0.5*I33*psidot0*psidot0*cos(phi0)*cos(phi0);




%%
%create function
therhs = @(t,z) bicycle_rhs(t,z,p);

%solving parameters
start = 0; stop = 20; t = linspace(start,stop,10000000);

%solve ode
small   = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small); 
soln    = ode45(therhs,t, z0,options);


%%
%%%%%%%%%%%%%%plotting%%%%%%%%%%%%%%

plot_soln(soln,start,stop,p)

%%%%%%

