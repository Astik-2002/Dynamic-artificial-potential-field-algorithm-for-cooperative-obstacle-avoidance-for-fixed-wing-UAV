%% initilization %%
close all
[x,y] = meshgrid(linspace(-200,200));
x_pos = -80;
y_pos = -80;
r_pos = [x_pos;y_pos];
x_goal = 200;
y_goal = 200;
k_att = 0.008;
k_rep = 10;
t = 0:0.01:4;
path = [];
dt = 0.01;
a = 6;
b = 1;
x_ob1 = -10;
y_ob1 = -10;
x_ob2 = 69;
y_ob2 = 56;
V_UAV = 1;
vx = 1;
vy = 1;
v = [vx;vy];
v_ob1 = [-0.2;-0.2];
v_ob2 = [0;0];
v_wp = [0;0];
fullscreen()
gif('morphing potential_heading.gif')
%% Simulation loop %%
for i = 1:length(t)
    path(i,1) = x_pos;
    path(i,2) = y_pos;
    %sqrt((x-10-v3(1)).^2+(y-10-v3(2)).^2) 
    
    x_ob1 = x_ob1 + v_ob1(1).*t(i);  %pos update of obstacle1
    y_ob1 = y_ob1 + v_ob1(2).*t(i);
    
    x_ob2 = x_ob2 + v_ob2(1).*t(i);  %pos update of obstacle1
    y_ob2 = y_ob2 + v_ob2(2).*t(i);

    x_goal = x_goal + v_wp(1).*t(i);
    y_goal = y_goal + v_wp(2).*t(i);
    
    X1 = x_pos - x_ob1;
    Y1 = y_pos - y_ob1;
    
    X2 = x_pos - x_ob2;
    Y2 = y_pos - y_ob2;
    
    
    X_ob1 = x-x_ob1;
    Y_ob1 = y-y_ob1;
    
    X_ob2 = x-x_ob2;
    Y_ob2 = y-y_ob2;
    
    X_goal = x_pos-x_goal;
    Y_goal = y_pos-y_goal;
    
    X0 = x-x_goal;
    Y0 = y-y_goal;
    
    theta = atan(abs(vy/vx));
    theta2 = atan(Y1/X1);
    %theta = 3.*t(i);
    mpf_goal = k_att.*(X0.^2+Y0.^2);
    %mpf_goal = 0;
    %r1 = (1+((X1.*cos(theta)+Y1.*sin(theta))./a).^2+((X1.*sin(theta)-Y1.*cos(theta))./b).^2).^(-1);
    mpf = k_rep./(((X_ob1.*cos(theta)+Y_ob1.*sin(theta))./10).^2+((X_ob1.*sin(theta)-Y_ob1.*cos(theta))./5).^2+1)+k_rep./(((X_ob2.*cos(theta)+Y_ob2.*sin(theta))./10).^2+((X_ob2.*sin(theta)-Y_ob2.*cos(theta))./5).^2+1);%+ mpf_goal;
    r1_grad = (1+(X1/a)^2+(Y1/b)^2);
    r2_grad = (1+(X2/a)^2+(Y2/b)^2);
    %d = sqrt(X1.^2+Y1.^2);
    %disp(d)
    vx = k_rep*((Y1*sin(theta)/(b^2))-(X1*cos(theta)/(a^2)))/(r1_grad)+10*((Y2*sin(theta)/(b^2))-(X2*cos(theta)/(a^2)))/(r2_grad)+k_att*(X_goal);
    vy = -k_rep*((Y1*cos(theta)/(b^2))+(X1*sin(theta)/(a^2)))/(r1_grad)-10*((Y2*cos(theta)/(b^2))+(X2*sin(theta)/(a^2)))/(r2_grad)+k_att*(Y_goal);
    [f1,f2] = gradient(mpf);
    quiver(x,y,-1.*f1,-1.*f2)
    %surf(x,y,mpf)
    %view(0,90)
    
    R = sqrt(vx^2+vy^2);
    
    a = vx/R;
    b = vy/R;
    
    
    x_pos = x_pos - V_UAV.*a.*t(i);
    y_pos = y_pos - V_UAV.*b.*t(i);
    hold on
    contour(x,y,mpf)
    plot3(x_goal,y_goal,0,'o','MarkerFaceColor','b')
    plot3(x_ob1,y_ob1,0,'o','MarkerFaceColor','r')
    plot3(x_ob2,y_ob2,0,'o','MarkerFaceColor','r')
    plot3(x_pos,y_pos,0,'o','MarkerFaceColor','y') %for quiver plot
    %plot([xp1 xp2],[yp1 yp2]);
    %view(0,90)
    drawnow
    pause(0.01)
    hold off
    gif
end
figure 
plot(path(:,1),path(:,2))
