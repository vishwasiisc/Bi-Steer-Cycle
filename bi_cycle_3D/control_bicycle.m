function[Tr,thetadot] = control_bicycle(t,z,p)

x = z(1);
y = z(2);
V = z(3);
phi = z(5);
phidot = z(6);

Kp = 50;
Kd = 80;


%%%%%%%%%%%%intresting thinngs you can do%%%%%%%%%%%%%

thetadot =-(Kp*phi + Kd*phidot);%0.5*(cos(t/2));%-0.5*sin(t)+0.5*cos(t/2);

Tr =0;% -0.1*V;

end