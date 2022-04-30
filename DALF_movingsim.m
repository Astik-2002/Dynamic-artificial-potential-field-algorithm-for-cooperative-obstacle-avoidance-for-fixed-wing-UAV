%dALF-LQR algorithm for co-operative obstacle avoidance and path planning in a dynamic
%environment for micro multirotor UAV. 
%Basic algorithm for obstacle aviodance

%og mpc = 5.*exp(-((x-1).^2+(y-3).^2)/0.5);
[x,y] = meshgrid(linspace(-50,50,100));
x_pos = -20;
y_pos = -20;
x_stat = -10;
y_stat = -5;
x_goal = 20;
y_goal = 20;
k_att = 1;
k_rep = 15;
t = 0:0.01:2;
theta_1 = 0;
theta_2 = 0;
path = [];
dt = 0.01;
a = 0.05;
b = 0.09;
%gif('obstacle_avoidance_quiver_morphed.gif')
for i = 2:length(t)
    %sqrt((x-10-v3(1)).^2+(y-10-v3(2)).^2) 
    path(i,1) = x_pos;
    path(i,2) = y_pos;
    v_ob1 = [7;9];
    v_ob2 = [0;-25];
    v_wp = [10;-28];
    v1 = [7.*t(i),9.*t(i)];           %velocity of obstacle 1
    v2 = [0.*t(i),-25.*t(i)];         %velocity of obstacle 2
    v3 = [-10.*t(i),2.*t(i)];          %velocity of waypoint
    %morphed potential function
    %z = 30.*(a.*((x-2-v1(1)).*cos(theta_1)+(y-2-v1(2)).*sin(theta_1)).^2+b.*((x-2-v1(1)).*sin(theta_1)-(y-2-v1(2)).*cos(theta_1)).^2+1).^(-1) + k_rep.*(a.*((x-15-v2(1)).*cos(theta_2)+(y-15-v2(2)).*sin(theta_2)).^2+b.*(((x-15-v2(1)).*sin(theta_2)-(y-15-v2(2)).*cos(theta_2)).^2+1)).^(-1) + k_att.*sqrt((x-x_goal-v3(1)).^2+(y-y_goal-v3(2)).^2)+ 15.*(0.5.*((x-x_stat).^2+(y-y_stat).^2)+1).^(-1);
    %non-morphed potential
    z = k_rep.*(0.5.*((x-2-v1(1)).^2+(y-2-v1(2)).^2)+1).^(-1) + k_rep.*(0.5.*((x-15-v2(1)).^2+(y-15-v2(2)).^2)+1).^(-1) + k_att.*sqrt((x-x_goal-v3(1)).^2+(y-y_goal-v3(2)).^2)+ k_rep.*(0.5.*((x-x_stat).^2+(y-y_stat).^2)+1).^(-1);
    [f1,f2] = gradient(z);
    f3 = -1.*f1;
    f4 = -1.*f2;
    fy = -k_rep.*(2.*(y_pos-2-v1(2)))./((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2+1).^2 -k_rep.*(2.*(y_pos-15-v2(2)))./((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2+1).^2 + k_att.*(y_pos-y_goal-v3(2))./(sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2))-15.*(2.*(y_pos-y_stat))./((x_pos-x_stat).^2+(y_pos-y_stat).^2+1).^2;
    fx = -k_rep.*(2.*(x_pos-2-v1(1)))./((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2+1).^2 -k_rep.*(2.*(x_pos-15-v2(1)))./((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2+1).^2 + k_att.*(x_pos-x_goal-v3(1))./(sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2))-15.*(2.*(x_pos-x_stat))./((x_pos-x_stat).^2+(y_pos-y_stat).^2+1).^2;
    y_pos = y_pos - fy.*t(i);
    x_pos = x_pos - fx.*t(i);
    z_pos = k_rep.*(0.5.*((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2)+1).^(-1) + k_rep.*(0.5.*((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2)+1).^(-1) + k_att.*sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2)+ 15.*(0.5.*((x_pos-x_stat).^2+(y_pos-y_stat).^2)+1).^(-1);
    x_goal1 = x_goal + v3(1);
    y_goal1 = y_goal + v3(2);
    v_uav = [(path(i,1)-path(i-1,1))./dt;(path(i,2)-path(i-1,2))./dt];
    v_rel1 = v_uav-v_ob1;
    v_rel2 = v_uav-v_ob2;
    theta_1 = atan(v_rel1(2)./v_rel1(1));
    theta_2 = atan(v_rel2(2)./v_rel2(1));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    quiver(x,y,f3,f4)
    %surfc(x,y,z)
    %contour(z)
    hold on 
    %plot3(x_pos,y_pos,z_pos,'o','MarkerFaceColor','y') %for surface plot
    plot3(x_pos,y_pos,0,'o','MarkerFaceColor','y') %for quiver plot
    %view(60,60)
    plot3(x_goal1,y_goal1,0,'o','MarkerFaceColor','b')
    hold off
    drawnow
    pause(0.01)
    %gif
end
