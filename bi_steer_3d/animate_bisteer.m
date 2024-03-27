function animate(soln,tend,speed,p)

h = p.h;
r = h/10;


theta_c=0:0.02:2*pi;   %angle arry for circle
wheel_R=r*[cos(theta_c);sin(theta_c)]';

tspan=linspace(0,tend,100000);
%skate=[linspace(0.1*d,-1.2*d,5);zeros(1,5)];

figure(4)
figure(5)
tic
t1=toc*speed;

zarray = deval(soln,tspan);
x_array = zarray(1,:);
y_array = zarray(2,:);
%{
lim_x_mx = max(x_array);
lim_x_mn = min(x_array);
lim_y_mx = max(y_array);
lim_y_mn = min(y_array);
%}

hold on

while t1<tend

    clf

    z=deval(soln,t1);
    x=z(1);
    y=z(2);
    

    hold on
    plot(x_array,y_array)
    plot(x,y,'r.',MarkerSize=16)
    axis equal
    
    plot(x,y,'r.')
    drawnow
    

    t1=toc*speed;

end



end
