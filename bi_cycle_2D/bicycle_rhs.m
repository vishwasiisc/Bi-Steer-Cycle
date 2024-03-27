function zdot = bicycle_rhs(t,z,p);

V = z(3);
psi = z(4);
theta = z(5);
%psidot = z(6);


%unpacking p
I33 = p.I33; df = p.df; dr = p.dr; m = p.m;


%%
%%%%%%%%%%%%%%%%%%%%%%%intresting things%%%%%%%%%%%%%%%%%%%%%%

[Tr,thetadot] = control_bicycle(t,z,p);

statedot = bicycle_dynamics(I33,Tr,V,df,dr,m,psi,theta,thetadot);
xdot = statedot(1);
ydot = statedot(2);
Vdot = statedot(3);
psidot = statedot(4);
%psiddot = statedot(5);


zdot = [xdot, ydot, Vdot, psidot,thetadot]';
end