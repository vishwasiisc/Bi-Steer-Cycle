function [Tf, Tr, theta_F, theta_R, theta_Fdot, theta_Rdot] = controller_bisteer3D(t,z,p)

phi = z(5);
phidot = z(6);


Kp = 10;
Kd = 1;

theta_F = 0;%-(Kp*phi+Kd*phidot);%-Kp*phi;
theta_R = 0;%(Kp*phi+Kd*phidot);%sin(t);

theta_Fdot =0;%0.5*cos(t);%-Kd*phidot;%cos(t);
theta_Rdot =0;%0.5*sin(t);

Tr = 0;%-(Kp*phi+Kd*phidot);
Tf = 0;%-(Kp*phi+Kd*phidot);

end