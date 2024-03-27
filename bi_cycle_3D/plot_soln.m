function plot_soln(soln,start,stop,p)
close all

df=p.df; dr=p.dr; h=p.h; m=p.m; I11=p.I11; I22=p.I22; I33=p.I33; g=p.g;

t = linspace(start, stop, 100000);

zsol = deval(soln,t);
%Z0 = zsol(:,1);

x   = zsol(1,:);
y   = zsol(2,:);
V   = zsol(3,:);
phi = zsol(5,:);
phidot    = zsol(6,:);
theta     = zsol(7,:);
psidot    = (V.*tan(theta))/(df+dr);

%%
E =   m*g*h*cos(phi)...
    + 0.5*m*(V.^2 + dr^2*(psidot.^2)...
    + h^2*(phidot.^2) + h^2*(psidot.^2)...
    - h^2*(psidot.^2).*(cos(phi).^2)...
    + 2*h*((V).*(psidot)).*(sin(phi))...
    - 2*dr*h*((phidot).*(psidot)).*(cos(phi)))...
    + 0.5*(I11*phidot.^2 + (I22*psidot.^2)/2 + (I33*psidot.^2)/2 ...
    - (I22*(psidot.^2).*(cos(2*phi)))/2 ...
    + (I33*(psidot.^2).*(cos(2*phi)))/2);


       %{
    +0.5*I11*(phidot.*phidot)...
    +0.5*I22*((psidot.*sin(phi))).*((psidot.*sin(phi)))...
    +0.5*I33*((psidot.*cos(phi))).*((psidot.*cos(phi)));
       %}

E0 = E(1);

dE = E-E0;

%%
%
figure(1)
hold on
plot(x,y);
plot(x(1),y(1),'b.',MarkerSize=13)
plot(x(end),y(end),'r.',MarkerSize=13)
legend('','start','end')
title(' Trajectory')
xlabel('x')
ylabel('y')
axis equal
movegui('northwest')
%}

%%
%
figure(2)
hold on
plot(t,V);
title('$ V$')
movegui('south')
%}

%%
%
figure(3)
plot(t,rad2deg(phi));
title('$\phi$')
movegui('southeast')
%}


%%
%
figure(4)
hold on
plot(t,dE);
%ylim([-5*small,5*small])
title('$\Delta E$')
movegui('south')
%}


end
