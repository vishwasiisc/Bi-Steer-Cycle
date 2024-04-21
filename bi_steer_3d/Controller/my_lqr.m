function [K,A,B] = my_lqr(t,z,p,state_ref,Q,R)

% unpacking parameters

I11 = p.I11; I22 = p.I22; I33 = p.I33; m = p.m; g = p.g;
h = p.h; df = p.df; dr = p.dr;

%% equb conditions

phi        = 0;
phidot     = 0;


Tf         = 0;
Tr         = 0;
theta_Fdot = 0;
theta_Rdot = 0;

%% Ref input


Vr       = state_ref(1);
theta_Fr = state_ref(2);
%theta_Fr = sin(t);
theta_Rr = state_ref(3);


A = A_matrix(I11,I22,I33,Tf,Tr,Vr,df,dr,g,h,m,phi,phidot,theta_Fr,theta_Rr,theta_Fdot,theta_Rdot);
B = B_matrix(I11,I22,I33,Vr,df,dr,h,m,phi,theta_Fr,theta_Rr);


K = lqr(A,B,Q,R);


%{
dynamics_lin = [phidot, Vdot, phiddot, theta_Fdot, theta_Rdot];
state_lin    = [   phi,    V,  phidot,    theta_F,    theta_R];
U            = [    Tf, Tr, theta_Fdot, theta_Rdot];

%}

end