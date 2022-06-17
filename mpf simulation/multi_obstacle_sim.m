%% initilization %%
close all
[x,y] = meshgrid(linspace(-200,200,100));
x_pos = -80;
y_pos = -80;
r_pos = [x_pos;y_pos];
x_goal = 60;
y_goal = 200;
k_att = 0.008;
t = 0:0.01:4;
path = [];
dt = 0.01;
a = 20;
b = 10;
num = 50;
% x_ob1 = -50;
% y_ob1 = -10;
% x_ob2 = 70;
% y_ob2 = 56;
obs = randi([-200 200],num,2);
v_ob = randi([-5 5],num,2)./5;
V_UAV = 2;
vx = 1;
vy = 1;
v = [vx;vy];
num_collision = 0;
v_wp = [0;0];
k_rep = 30*V_UAV;
vel_uav = [1;1];
v_rel = zeros(num,2);
fullscreen()
%gif('morphing potential_multiple_obstacles_100.gif')
%% Simulation loop %%
for i = 1:length(t)
    path(i,1) = x_pos;
    path(i,2) = y_pos;
     
    obs = obs + v_ob.*t(i);
 
    x_goal = x_goal + v_wp(1).*t(i);
    y_goal = y_goal + v_wp(2).*t(i);
    
    v_subx = ones(num,1).*vel_uav(1);
    v_suby = ones(num,1).*vel_uav(2);
    v_sub = [v_subx v_suby];
    
    v_rel = v_sub - v_ob;
    
    X_goal = x_pos - x_goal;
    Y_goal = y_pos - y_goal;
    
    mpf = morphing_potential(x,y,obs,v_rel,a,b,k_rep,x_goal,y_goal,k_att);
    
    d = sqrt(X_goal.^2+Y_goal.^2);
    if(d<=10)
        break;
    end
    %disp(d)
    v = mpf_multi_obs(x_pos,y_pos,obs,v_rel,a,b,k_rep,x_goal,y_goal,k_att);
    vx = v(1);
    vy = v(2);
    [f1,f2] = gradient(mpf);
    quiver(x,y,-1.*f1,-1.*f2)
    %surf(x,y,mpf)
    %view(0,90)
    
    R = sqrt(vx^2+vy^2);
    
    c = vx/R;
    s = vy/R;
    
    vel_uav = [-V_UAV*c;-V_UAV*s];
    
    x_pos = x_pos + vel_uav(1).*t(i);
    y_pos = y_pos + vel_uav(2).*t(i);
    
    num_collision = collision_report(x_pos,y_pos,obs);
    hold on
    contour(x,y,mpf)
    plot3(x_goal,y_goal,0,'o','MarkerFaceColor','g')
    plot3(obs(:,1),obs(:,2),zeros(num,1),'o','MarkerFaceColor','r')
    plot3(x_pos,y_pos,0,'o','MarkerFaceColor','y') %for quiver plot
    
    drawnow
    pause(0.01)
    hold off
    %gif
end
figure 
plot(path(:,1),path(:,2))
disp(num_collision)
