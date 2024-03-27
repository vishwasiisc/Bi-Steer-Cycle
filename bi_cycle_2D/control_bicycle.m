function[Tr,thetadot] = control_bicycle(t,z,p)

psi = z(4);
V   = z(3);


Kp = 0.1;
Kd = 0.1;

%%%%%%%%%%%%intresting thinngs you can do%%%%%%%%%%%%%

thetadot = -0.5*sin(t)+0.5*cos(t/2);

Tr = 0;

end