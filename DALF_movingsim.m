%dALF-LQR algorithm for co-operative obstacle avoidance and path planning in a dynamic
%environment for micro multirotor UAV. 
%Basic algorithm for obstacle aviodance
%plotting the potential for a bell-curve function

%og mpc = 5.*exp(-((x-1).^2+(y-3).^2)/0.5);
[x,y] = meshgrid(linspace(-30,30,100));
x_pos = -20;
y_pos = -20;
x_stat = -10;
y_stat = -5;
x_goal = 20;
y_goal = 20;
k_att = 1;
k_rep = 10;
t = 0:0.01:4;
for i = 1:length(t)
    v1 = [7.*t(i),9.*t(i)];           %velocity of obstacle 1
    v2 = [0.*t(i),-25.*t(i)];         %velocity of obstacle 2
    v3 = [0.*t(i),0.*t(i)];          %velocity of waypoint(currently 0)
    %sqrt((x-10-v3(1)).^2+(y-10-v3(2)).^2) 
    z = k_rep.*(0.5.*((x-2-v1(1)).^2+(y-2-v1(2)).^2)+1).^(-1) + k_rep.*(0.5.*((x-15-v2(1)).^2+(y-15-v2(2)).^2)+1).^(-1) + k_att.*sqrt((x-x_goal-v3(1)).^2+(y-y_goal-v3(2)).^2)+ 15.*(0.5.*((x-x_stat).^2+(y-y_stat).^2)+1).^(-1);
    [f1,f2] = gradient(z);
    f3 = -1.*f1;
    f4 = -1.*f2;
    fy = -k_rep.*(2.*(y_pos-2-v1(2)))./((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2+1).^2 -k_rep.*(2.*(y_pos-15-v2(2)))./((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2+1).^2 + k_att.*(y_pos-y_goal-v3(2))./(sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2))-15.*(2.*(y_pos-y_stat))./((x_pos-x_stat).^2+(y_pos-y_stat).^2+1).^2;
    fx = -k_rep.*(2.*(x_pos-2-v1(1)))./((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2+1).^2 -k_rep.*(2.*(x_pos-15-v2(1)))./((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2+1).^2 + k_att.*(x_pos-x_goal-v3(1))./(sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2))-15.*(2.*(x_pos-x_stat))./((x_pos-x_stat).^2+(y_pos-y_stat).^2+1).^2;
    y_pos = y_pos - fy.*t(i);
    x_pos = x_pos - fx.*t(i);
    z_pos = k_rep.*(0.5.*((x_pos-2-v1(1)).^2+(y_pos-2-v1(2)).^2)+1).^(-1) + k_rep.*(0.5.*((x_pos-15-v2(1)).^2+(y_pos-15-v2(2)).^2)+1).^(-1) + k_att.*sqrt((x_pos-x_goal-v3(1)).^2+(y_pos-y_goal-v3(2)).^2)+ 15.*(0.5.*((x_pos-x_stat).^2+(y_pos-y_stat).^2)+1).^(-1);

    X = ['fx = ',fx];
    Y = ['fy = ',fy];
    disp('vx at 10,10 =')
    disp(fx)
    disp('vy at 10,10 =')
    disp(fy)
    disp('vx at goal =')
    disp(f3(20,20))
    disp('vy at goal =')
    disp(f4(20,20))
    %quiver(x,y,f3,f4)
    surf(x,y,z)
    view(60,60)
    hold on 
    plot3(x_pos,y_pos,z_pos,'o','MarkerFaceColor','y') %for surface plot
  %  plot3(x_pos,y_pos,0,'o','MarkerFaceColor','y') %for quiver plot
    plot3(x_goal,y_goal,0,'o','MarkerFaceColor','b')
    hold off
    drawnow
    pause(0.01)
    if(x_pos == x_goal)
        if(y_pos == y_goal)
            disp('success')
        end
    end
end
