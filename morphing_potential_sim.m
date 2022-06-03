%% initilization %%
close all
[x,y] = meshgrid(linspace(-200,200));
x_pos = 0;
y_pos = 0;
r_pos = [x_pos;y_pos];
x_goal = 100;
y_goal = 100;
k_att = 0.1;
k_rep = 5000000;
t = 0:0.01:5;
path = [];
dt = 0.01;
a = 6;
b = 1;
x_ob1 = 60;
y_ob1 = 60;
x_ob2 = 42;
y_ob2 = 56;

vx = 1;
vy = 1;
v = [vx;vy];
gif('morphing potential original.gif')
%% Simulation loop %%
for i = 1:length(t)
    path(i,1) = x_pos;
    path(i,2) = y_pos;
    %sqrt((x-10-v3(1)).^2+(y-10-v3(2)).^2) 
    v_ob1 = [-0;-0];
    x_ob1 = x_ob1 + v_ob1(1);  %pos update of obstacle1
    y_ob1 = y_ob1 + v_ob1(2);
    
    
    
    X = x_pos - x_ob1;
    Y = y_pos - y_ob1;
    X1 = x-x_ob1;
    Y1 = y-y_ob1;
    
    X_goal = x_pos-x_goal;
    Y_goal = y_pos-y_goal;
    
    X2 = x-x_goal;
    Y2 = y-y_goal;
    
    theta = atan(vy/vx);
    theta2 = atan(Y/X);
    %theta = 3.*t(i);
    mpf_goal = 0.001.*(X2.^2+Y2.^2);
    %mpf_goal = 0;
    %r1 = (1+((X1.*cos(theta)+Y1.*sin(theta))./a).^2+((X1.*sin(theta)-Y1.*cos(theta))./b).^2).^(-1);
    mpf = 100./(((X1.*cos(theta)+Y1.*sin(theta))./a).^2+((X1.*sin(theta)-Y1.*cos(theta))./b).^2+1);%mpf_goal;
    r1_grad = (1+(X/a)^2+(Y/b)^2);
    d = sqrt(X.^2+Y.^2);
    disp(d)
    if d<=100
        vx = 12*((Y*sin(theta)/(b^2))-(X*cos(theta)/(a^2)))/(r1_grad^2)+0.005*(X_goal);
        vy = -12*((Y*cos(theta)/(b^2))+(X*sin(theta)/(a^2)))/(r1_grad^2)+0.005*(Y_goal);
    else
        vx = 0.005*X_goal;
        vy = 0.005*Y_goal;
    end
    [f1,f2] = gradient(mpf);
    quiver(x,y,-1.*f1,-1.*f2)
    %surf(x,y,mpf)
    %view(0,90)
    x_pos = x_pos - vx.*t(i);
    y_pos = y_pos - vy*t(i);
    hold on
    plot3(x_goal,y_goal,0,'o','MarkerFaceColor','b')
    plot3(x_ob1,y_ob1,0,'o','MarkerFaceColor','r')
    plot3(x_pos,y_pos,0,'o','MarkerFaceColor','y') %for quiver plot
    %plot([xp1 xp2],[yp1 yp2]);
    %view(0,90)
    drawnow
    pause(0.01)
    hold off
    gif
end

