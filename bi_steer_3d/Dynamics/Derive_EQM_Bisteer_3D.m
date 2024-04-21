clc
clear

syms m I11 I22 I33 real
syms dr df h       real

syms    x    y    psi    phi    theta_R    theta_F real
syms xdot ydot psidot phidot theta_Rdot theta_Fdot real

syms Tr   Tf                 real
syms V  Vdot vFscalar        real
syms Nf Nr Nfg Nrg g         real 
syms phiddot psiddot         real




state    = [   x,    y,    V,    psi,  psidot,     phi,   phidot,    theta_R,    theta_F];
statedot = [xdot, ydot, Vdot, psidot, psiddot,  phidot,  phiddot, theta_Rdot, theta_Fdot];


%%

i = [1,0,0]'; 
j = [0,1,0]'; 
k = [0,0,1]';

a1 =  cos(psi)*i + sin(psi)*j;
a2 = -sin(psi)*i + cos(psi)*j;
a3 =  k;


%% Body frame
b1 =  a1;
b2 =  cos(phi)*a2 + sin(phi)*k;
b3 = -sin(phi)*a2 + cos(phi)*k;


%% front wheel frame
f1 =  cos(theta_F)*b1 + sin(theta_F)*b2;
f2 = -sin(theta_F)*b1 + cos(theta_F)*b2;
f3 = b3;


%% rear wheel frame
r1 =  cos(theta_R)*b1 + sin(theta_R)*b2;
r2 = -sin(theta_R)*b1 + cos(theta_R)*b2;
r3 = b3;

%% wheel ground track  
f_thred  = cross(f2,k);
f_constr = cross(k,f_thred);


r_thred  = cross(r2,k);
r_constr = cross(k,r_thred);


%inertia tensor
IG = [I11,   0, 0;
        0, I22, 0;
        0,   0, I33];


%inertia tensor is along the princple body axis
%needs to be transformed into the inertial frame of refrance to write
%dynamics in inertial frame
B=[b1,b2,b3];
IG = B*IG*B';
IG = simplify(IG);


%%

rGrelR =          dr*a1 + h*b3;
rFrelR =     (dr+df)*a1;
rFrelG =          df*a1 - h*b3;

rGrelF = -rFrelG;
rRrelF = -rFrelR;

omega  = psidot*k + phidot*a1;
vR     = V*r_thred;

xdot   = vR(1);
ydot   = vR(2);



%%
%%%%%%%%%%%%%%%%calculation to find psiddot%%%%%%%%%%%%%%%%%%%%%
%psiddot depends on front vF and vR


%side calculation to find vf

vFa     = vR + cross(omega,rFrelR);  %front point velocity from rear veocity
vFa     = simplify(vFa);              
vFb     = vFscalar*f_thred;               %front point velocity from skate constraint


eqn1    = dot(vFa-vFb,i);
eqn2    = dot(vFa-vFb,j);
eqns    = [eqn1, eqn2];

[psidot,vFscalar] = solve(eqns,[psidot,vFscalar]);  %solving eqns to find heading rate and vF


psidot            = simplify(psidot);
vFscalar          = simplify(vFscalar);

vF                = vFscalar*f_thred;
vF                = simplify(vF);

psiddot           = jacobian(psidot,state)*statedot';  %psiddot in terms of Vdot, theta_F, theta_R
psiddot           = simplify(psiddot);

%%
%%%%calculation to find accleration of CG
%

%omega  = subs(omega);

vGrelR = cross(omega,rGrelR);
vG     = vR + vGrelR;
%vG     = subs(vG);
vG     = simplify(vG);

aR     = jacobian(vR,state)*statedot';
aGrelR = jacobian(vGrelR,state)*statedot';


aG     = aR + aGrelR;
aG     = subs(aG);   %substitute psidot in aG new aG in terms of theta_R F
aG     = simplify(aG);


alpha   = jacobian(omega,state)*statedot';  % omega cross omega =0

%%%%%%above this line all seems correct and checked 23 feb 2024



%%
%{

%%
%%%%%%%%%%%%matlabfunction to create a function file from formula

%statedot = subs(statedot);

%matlabFunction(statedot,'File','Bi_steer_3D_DAE.m','Optimize',true);

%%

%}

%linear momentum balance about cg 
%

LMB_CG = Tf*f_thred + Nf*f_constr + Tr*r_thred + Nr*r_constr + Nfg*k + Nrg*k -m*g*k - m*aG;

%angular momentum balance about fromt point

%bottom part requires attension EQN3
%

M_F    =     cross(rRrelF, Nr*r_constr) + cross(rRrelF,Tr*r_thred)...
          +  cross(rGrelF,-m*g*k) + cross(rRrelF,Nrg*k) ;


H_dot_F = cross(rGrelF,aG*m) + IG*alpha + cross(omega,IG*omega);

%{
M_R    =    cross(rFrelR, Nf*f2)  + cross(rFrelR,Tf*f1)...
         +  cross(rGrelR,-m*g*k)  + cross(rFrelR,Nfg*k);

H_dot_R = cross(rGrelR,aG)*m  + IG*alpha;
%}




AMB_F   = M_F - H_dot_F;
AMB_F   = simplify(AMB_F);
%{
AMB_R   = M_R - H_dot_R;
AMB_R   = simplify(AMB_R);
%}

EQN1 = dot(LMB_CG,i);
EQN1 = subs(EQN1);
EQN1 = simplify(EQN1);

EQN2 = dot(LMB_CG,j);
EQN2 = subs(EQN2);
EQN2 = simplify(EQN2);

EQN3 = dot(AMB_F,k);
EQN3 = subs(EQN3);
EQN3 = simplify(EQN3);

EQN4 = dot(AMB_F,a1);
EQN4 = subs(EQN4);
EQN4 = simplify(EQN4);

%
EQNS = [EQN1 EQN2 EQN3 EQN4];
VARS = [Vdot, phiddot, Nf, Nr];

[Vdot,phiddot,Nf,Nr] = solve(EQNS,VARS);
Vdot = simplify(Vdot);





%%

Vdot    = subs(Vdot);
Vdot    = simplify(Vdot);
%
phiddot = subs(phiddot);
phiddot = simplify(phiddot);

psiddot = subs(psiddot);
psiddot = simplify(psiddot);

%}



%

statedot = subs(statedot)';
statedot = simplify(statedot);






%matlabFunction(statedot,"File","bi_steer_3D_Dynamics_full.m",Optimize=true);

dynamics_lin = [phidot, Vdot, phiddot, theta_Fdot, theta_Rdot];
state_lin    = [   phi,    V,  phidot,    theta_F,    theta_R];
U            = [    Tf, Tr, theta_Fdot, theta_Rdot];


A_linear = jacobian(dynamics_lin,state_lin);
B_linear = jacobian(dynamics_lin, U);


matlabFunction(A_linear,'File','A_matrix.m','Optimize',true);
matlabFunction(B_linear,'File','B_matrix.m','Optimize',true);



%}



%}
