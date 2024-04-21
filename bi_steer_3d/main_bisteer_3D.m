%main bisteer 3D

clear;
close all;
clc;

restoredefaultpath
addpath("Controller","Dynamics","Graphics");


dr = 0.1; df = 0.1;

I11 = 0.1; I22 = 0.1; I33 = 0.1;
  m = 1;   g = 10;    h = 0.1;

%%
%packing parameters
p.dr = dr; p.df = df;

p.I11 = I11; p.I22 = I22; p.I33 = I33;

p.m = m; p.g = g; p.h = h;

%%
% initial condition
x0 = 0;
y0 = 0;
V0 = 0;

psi0 = deg2rad(0);  %heading
phi0 = deg2rad(0);  %lean angle
phidot0 = 0; %lean rate

theta_F0 = deg2rad(0);  
theta_R0 = deg2rad(0);
 

z0 = [x0, y0, V0, psi0, phi0, phidot0, theta_F0, theta_R0]';

%%
%

%lqr stuff
V_phi      = 1;
theta_Fr = deg2rad(0);
theta_Rr = deg2rad(0);

ref_phi = [V_phi,theta_Fr,theta_Rr];
Q_phi = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_phi = diag([1e1, 1e1, 1e2, 1e2]);



ref_V   = [ 0.01,deg2rad(0),deg2rad(0)];
Q_V = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_V = diag([1e3, 1e3, 1e2, 1e2]);



ref_theta_F = [0.0,deg2rad(90),deg2rad(0)];
Q_F = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_F = diag([1e2, 1e2, 1e1, 1e1]);

ref_theta_R = [0.0,deg2rad(0),deg2rad(90)];
Q_R = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_R = diag([1e2, 1e2, 1e1, 1e1]);

ref_3 = [0.0,deg2rad(1),deg2rad(0)];
Q_3 = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_3 = diag([1e2, 1e2, 1e1, 1e1]);

ref_4 = [1,deg2rad(1),deg2rad(0)];
Q_4 = diag([1e4, 1e2, 1e4, 1e1, 1e1]);
R_4 = diag([1e2, 1e2, 1e1, 1e1]);





p.ref_phi = ref_phi;
p.ref_V   = ref_V;
p.ref_theta_F = ref_theta_F;
p.ref_theta_R = ref_theta_R;
p.ref_3 = ref_3;
p.ref_4 = ref_4;

Q = diag([1e4, 1e3, 1e4, 1e1, 1e1]);
R = diag([1e0, 1e0, 1e1, 1e1]);

p.Q = Q;
p.R = R;
%
[K_phi,~,~] = my_lqr(0,z0,p,ref_phi,Q_phi,R_phi);   %z0 not used
[K_V,~,~]  = my_lqr(0,z0,p,ref_V,Q_V,R_V);
[K_theta_F,~,~] = my_lqr(0,z0,p,ref_theta_F,Q_F,R_F);
[K_theta_R,~,~] = my_lqr(0,z0,p,ref_theta_R,Q_R,R_R);
[K_3,~,~] = my_lqr(0,z0,p,ref_3,Q_3,R_3);
[K_4,~,~] = my_lqr(0,z0,p,ref_4,Q_4,R_4);


p.K_phi     = K_phi;
p.K_V       = K_V;
p.K_theta_F = K_theta_F;
p.K_theta_R = K_theta_R;
p.K_3 = K_3;
p.K_4 = K_4;



 
%}

%%
%{
eqn1 = simplify(subs(Vdot,   [phidot,theta_Fdot,theta_Rdot],[0,0,0]));
eqn2 = simplify(subs(phiddot,[phidot,theta_Fdot,theta_Rdot],[0,0,0]));

%

eqn1 = simplify(subs(eqn1,[phi,Tf,Tr],[0.01,0,0]));
eqn2 = simplify(subs(eqn1,[phi,Tf,Tr],[0.01,0,0]));

%matlabFunction([eqn1,eqn2],'File','equb.m',Optimize=true)
fun = @ equb;
theta = [0.1,0.02];

theta=fsolve(fun,theta)

%}



%%

%
%creat a function
therhs = @(t,z) bisteer_3D_rhs(t,z,p);
fall   = @(t,z) bicycle_fall(t,z,p);

%solving parameters
start = 0; stop = 80; t = linspace(start,stop,1000000);

%solve ode
small   = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small, Events=fall);
soln    = ode45(therhs,t, z0,options);



%%
%plotting

save = 0;
speed = 1;
animate_bisteer(soln,stop,p,speed,save)


plot_soln(soln,start,stop,p)



