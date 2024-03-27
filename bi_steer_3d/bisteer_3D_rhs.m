function zdot = bisteer_3D_rhs(t,z,p)

%unpacking state vector z

%{
interval_noise = 1;
if rem(t,interval_noise) == 0
    noise = deg2rad(100*rand-50)

else
    noise = 0;
end

%}


V       = z(3);
psi     = z(4);
phi     = z(5);
phidot  = z(6);

%rem(t,interval_noise)





%unpacking parameters

I11 = p.I11; I22 = p.I22; I33 = p.I33; m = p.m; g = p.g;
h = p.h; df = p.df; dr = p.dr;



[Tf, Tr, theta_F, theta_R, theta_Fdot, theta_Rdot] = controller_bisteer3D(t,z,p);

theta_R = z(7);
theta_F = z(8);


%Vdot = 0;
%statedot = Bi_steer_3D_DAE(I11,V,Vdot,df,dr,g,h,m,phi,phidot,psi,theta_F,theta_R,theta_Fdot,theta_Rdot);

statedot = bi_steer_3D_Dynamics_full(I11,I22,I33,Tf,Tr,V,df,dr,g,h,m,phi,phidot,psi,theta_F,theta_R,theta_Fdot,theta_Rdot);

xdot     = statedot(1);
ydot     = statedot(2);
Vdot     = statedot(3);
psidot   = statedot(4);
%phidot   =  
phiddot  = statedot(7);


zdot = [xdot,ydot,Vdot,psidot,phidot,phiddot,theta_Rdot,theta_Fdot]';


end