%derive bicycle eqm 1st march


clear;
clc;

syms I33 m   positive
syms h dr df positive
syms x y V psi theta real
syms xdot ydot Vdot psidot thetadot real
syms psiddot thetaddot real
syms xG yG xdotG ydotG real
syms xddot yddot aR real

state = [x, y, V, psi,psidot,theta]';
statedot = [xdot, ydot, Vdot, psidot,psiddot,thetadot]';

%%
%useful states that we want
zdot = [xdot, ydot, Vdot, psidot]';
%zdot = [xdot,ydot,aR]';


%%
%frames

i = [1,0,0]';
j = [0,1,0]';
k = [0,0,1]';

a1 =  cos(psi)*i + sin(psi)*j;
a2 = -sin(psi)*i + cos(psi)*j;

f1 =  cos(theta)*a1 + sin(theta)*a2;
f2 = -sin(theta)*a1 + cos(theta)*a2;

%%
%velocity
omega = psidot*k;

vR = V*a1;
xdot = vR(1);
ydot = vR(2);


%%
%accleration
rGrelR = dr*a1;
vGrelR = cross(omega,rGrelR);

vG = vR +vGrelR;

aR = jacobian(vR,state)*statedot;

%aR =Vdot*a1;


aGrelR = jacobian(vGrelR,state)*statedot;
aG = aR + aGrelR;

aG1 = Vdot*a1 + V*psidot*a2 + dr*psiddot*a2 - dr*psidot*psidot*a1;

alpha = jacobian(omega,state)*statedot;

%

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

eqn1 = dot(vFa-vFb,i);
eqn2 = dot(vFa-vFb,j);
eqns = [eqn1, eqn2];
vars = [vFscalar,psidot];

[vFscalar,psidot] = solve(eqns,vars); 
psiddot = simplify(jacobian(psidot,state)*statedot);

%%
%

alpha   = subs(alpha);
alpha   = simplify(alpha);

%
aG = subs(aG);
aG = simplify(aG);


%%
%%%%%%%%%%%%%%%%anshuman das%%%%%%%%%%%%%%%%%
%{
syms Tr real

q1 = (df+dr)/tan(theta);
q2 = (df +dr)/sin(theta);

rCrelR = q1*a2;
rCrelF = q2*f2;
rRrelC = -rCrelR;

rGrelC = rGrelR + rRrelC;

M_C = cross(rRrelC,Tr*a1);
H_dot_C = cross(rGrelC,aG)*m + I33*alpha;

AMB_C = M_C-H_dot_C;
AMB_C =simplify(AMB_C);

eq = dot(AMB_C,k);
Vdot = solve(eq,Vdot);
%}







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%}
%


%
%%
%angular momentum balance and linear momentum bolance to find relation
%between accleration and forces
syms Tr Nr Nf real
rRrelF  = - rFrelR;
rGrelF  = -df*a1;
rFrelG  = -rGrelF;
rRrelG  = -rGrelR;

LMB_CG = Tr*a1 + Nr*a2 + Nf*f2  - m*aG;
LMB_CG = simplify(LMB_CG);



M_F = cross(rRrelF,Tr*a1) + cross(rRrelF,Nr*a2);
M_R = cross(rFrelR,Nf*f2);

M_CG = cross(rFrelG,Nf*f2) + cross(rRrelG,Nr*a2) + cross(rRrelG,Tr*a1);

H_dot_F = cross(rGrelF,aG)*m + I33*alpha;
H_dot_R = cross(rGrelR,aG)*m + I33*alpha;
H_dot_CG = I33*alpha;


AMB_F = M_F - H_dot_F;
AMB_R = M_R - H_dot_R;
AMB_CG = M_CG - H_dot_CG;



EQN1 = simplify(dot(LMB_CG,a1));



EQN2 = simplify(dot(LMB_CG,a2));
EQN3 = simplify(dot(AMB_F,k));
EQN4 = simplify(dot(AMB_R,k));
EQN5 = simplify(dot(AMB_CG,k));

EQNS = [EQN1, EQN2, EQN5];
VARS = [Vdot, Nf, Nr];


[Vdot, Nf, Nr] = solve(EQNS,VARS);

%Vdot = subs(Vdot);
Vdot = simplify(Vdot);
psiddot = simplify(subs(psiddot));

%}

zdot = simplify(subs(zdot));

matlabFunction(zdot,"File",'bicycle_dynamics.m','Optimize',true);



