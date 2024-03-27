clc;
clear all;

syms m I33 dr df real
syms theta_F theta_R theta_Fdot theta_Rdot real
syms psi psidot psiddot real
syms x y V xdot ydot Vdot real

%%
state = [x, y, V, psi, theta_F, theta_R]';
statedot = [xdot, ydot, Vdot, psidot, theta_Fdot, theta_Rdot]

zdot = [xdot, ydot, Vdot, psidot]';
%%

