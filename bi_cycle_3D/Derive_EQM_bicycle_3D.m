%derive bicycle eqm 1st march


clear;
clc;

syms I11 I22 I33 m g  real
syms h dr df real
syms x y V psi theta real
syms xdot ydot Vdot psidot thetadot real
syms psiddot thetaddot real
syms xG yG xdotG ydotG real
syms xddot yddot  real
syms phi phidot phiddot real


state =    [   x,    y,    V,    psi,  psidot,    phi,  phidot,    theta]';
statedot = [xdot, ydot, Vdot, psidot, psiddot, phidot, phiddot, thetadot]';

%%
% states that we want
zdot = [xdot, ydot, Vdot, psidot, phiddot]';

%zdot = [xdot,ydot,aR]';


%%
%frames

i = [1,0,0]'; j = [0,1,0]'; k = [0,0,1]';

a1 =  cos(psi)*i + sin(psi)*j;
a2 = -sin(psi)*i + cos(psi)*j;
a3 = k;

f1 =  cos(theta)*a1 + sin(theta)*a2;
f2 = -sin(theta)*a1 + cos(theta)*a2;
f3 = k;

b1 = a1;
b2 =  cos(phi)*a2 + sin(phi)*a3;
b3 = -sin(phi)*a2 + cos(phi)*a3;


%%
%velocity
omega = psidot*k + phidot*b1;

vR = V*a1;
xdot = vR(1);
ydot = vR(2);


%%
%accleration
rGrelR = dr*a1 + h*b3;
vGrelR = cross(omega,rGrelR);

vG = vR + vGrelR;

aR = jacobian(vR,state)*statedot;

%aR =Vdot*a1;


aGrelR = jacobian(vGrelR,state)*statedot;
aG = aR + aGrelR;

alpha = jacobian(omega,state)*statedot;

%
%%
%%%%%%%%%%%%%%energy calculation%%%%%%%%%%%%

KE_L = 0.5*m*norm(vG)^2;
KE_L = simplify(KE_L);




%%
%%%%%%%%Note%%%%%%%%
%aR is not Vdot*a1 has a centripital comoponent in a2 direction

%none of the variable relationship is found yet

%%
%calculation to find psidot and psiddot
%psidot is related to theta
%skate constraint on the front wheel will give theta and psi psidot
%relation

syms vFscalar real

rFrelR = (df+dr)*a1;
vFrelR = cross(omega,rFrelR);


vFa = vFscalar*f1;
vFb = vR + vFrelR;

eqn3 = dot(vFa-vFb,i);
eqn2 = dot(vFa-vFb,j);
eqns = [eqn3, eqn2];
vars = [vFscalar,psidot];

[vFscalar,psidot] = solve(eqns,vars); 
psiddot = simplify(jacobian(psidot,state)*statedot);

%%
%substitution

alpha   = subs(alpha);
alpha   = simplify(alpha);

%
aG = subs(aG);
aG = simplify(aG);

%%%%%%%%%%%%%%% This compelets kinematics%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
%%
%
%angular momentum balance and linear momentum bolance to find relation
%between accleration and forces
syms Tr Nr Nf Ngf Ngr real

B = [b1,b2,b3];

IG = [I11,   0,  0;
        0, I22,  0;
        0,   0,I33];

IG = B*IG*B'; 

%

rRrelF  = - rFrelR;
rGrelF  = -df*a1 + h*b3;
rFrelG  = -rGrelF;
rRrelG  = -rGrelR;
%}
%%
%{
LMB_CG = Tr*a1 + Nr*a2 + Nf*f2 + Ngf*k +Ngr*k - m*g*k  - m*aG;
LMB_CG = simplify(LMB_CG);



M_F =  cross(rRrelF,Tr*a1) + cross(rRrelF,Nr*a2) + cross(rRrelF,Ngr*k)...
      +cross(rGrelF,-m*g*k);

H_dot_F = cross(rGrelF,aG)*m +  IG*alpha;% + cross(omega,IG*omega);



M_CG =  cross(rFrelG,Nf*f2) + cross(rRrelG,Nr*a2) + cross(rRrelG,Tr*a1)...
       +cross(rFrelG,Ngf*k) + cross(rRrelG,Ngr*k);

H_dot_CG = cross(omega,IG*omega) + IG*alpha;


AMB_F = M_F - H_dot_F;

AMB_CG = M_CG - H_dot_CG;



eqn3 = simplify(dot(AMB_F,a1));

%phiddot = solve(eqn3,phiddot)

eqn4 = simplify(dot(LMB_CG,a1));
eqn5 = simplify(dot(LMB_CG,a2));
eqn6 = simplify(dot(AMB_F,k));
%eqn7 = 


eqns2 = [eqn3, eqn4, eqn5, eqn6];
vars2 = [phiddot, Vdot, Nf, Nr];


[phiddot, Vdot, Nf, Nr] = solve(eqns2,vars2);

Vdot = subs(Vdot);
Vdot = simplify(Vdot);
psiddot = simplify(subs(psiddot));
phiddot = simplify(subs(phiddot));

%}

zdot = simplify(subs(zdot));

%matlabFunction(zdot,"File",'bicycle_dynamics_CG.m','Optimize',true);



