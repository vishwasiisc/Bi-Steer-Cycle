function zdot = bicycle_rhs(t,z,p)

V = z(3);
psi = z(4);
phi = z(5);
phidot = z(6);
theta = z(7);
%psidot = z(6);


%unpacking p
I33 = p.I33; df = p.df; dr = p.dr; m = p.m;
I11 = p.I11; I22 = p.I22; h =p.h;  g =p.g;


%%
%%%%%%%%%%%%%%%%%%%%%%%intresting things%%%%%%%%%%%%%%%%%%%%%%

[Tr,thetadot] = control_bicycle(t,z,p);

statedot = bicycle_dynamics(I11,I22,I33,Tr,V,df,dr,g,h,m,phi,phidot,psi,theta,thetadot);
%statedot = bicycle_dynamics_incorrect(I11,I22,I33,Tr,V,df,dr,g,h,m,phi,phidot,psi,theta,thetadot);
xdot = statedot(1);
ydot = statedot(2);
Vdot = statedot(3);
psidot = statedot(4);
phiddot = statedot(5);


zdot = [xdot, ydot, Vdot, psidot, phidot, phiddot, thetadot]';
end