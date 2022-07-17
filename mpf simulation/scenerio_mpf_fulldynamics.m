close all
gif('full_system_dynamics.gif')

x_pos = 0;
y_pos = 0;
path_UAV = [];
V = 15;
path_obs1 = [];
path_obs2 = zeros(300,3);
roll_angle = [];
heading_angle = [];
i = 0;
phi = pi/4;
k_rep0 = 30;
a = 0.5;
%% scenerio generation
scene = uavScenario("UpdateRate",100,"StopTime",3,"ReferenceLocation",[0 0 0]);
x_sob1 = 500;
y_sob1 = 550;

x_sob2 = 450;
y_sob2 = 500;

x_sob3 = 250;
y_sob3 = 250;

x_dob1 = 290;
y_dob1 = 800;

obs = [x_sob1, y_sob1;
       x_sob2, y_sob2;
       x_sob3, y_sob3
       x_dob1, y_dob1 
       ];
v_ob = [0, 0;
        0,0;
        0,0;
        0,-14];
x_goal = 700;
y_goal = 700;
phi_e = pi/2;
w2 = [x_goal;y_goal];
w1 = [x_pos;y_pos];
c = 0;
b = [11; 6; 9; 20];
color.Gray = 0.651*ones(1,3);
color.Green = [0.3922 0.8314 0.0745];
color.Red = [1 0 0];
color.White = [1 1 1];

addMesh(scene,"polygon",{[0 0; 800 0; 800 800; 0 800],[-4 0]},color.Gray)

% Adding obstacles.
addMesh(scene,"polygon",{polygon_generator(x_sob1,y_sob1,18,10),[0 50]},color.Red)
addMesh(scene,"polygon",{polygon_generator(x_sob2,y_sob2,20,10),[0 50]},color.Red)
addMesh(scene,"polygon",{polygon_generator(x_sob3,y_sob3,15,8),[0 50]},color.Red)
addMesh(scene,"cylinder",{[x_goal y_goal 5],[0 15]},color.Green)

% adding controlled UAV
plat = uavPlatform("Controlled_UAV",scene,"ReferenceFrame",'ENU',"InitialPosition",[x_pos y_pos 15],"InitialOrientation",eul2quat([pi/3 0 0]));      % angle orientation ( yaw, pitch, roll)
updateMesh(plat,"fixedwing",{10},color.Green,[0 0 0],eul2quat([0 0 0]))

% adding first dynamic obstacle
d_ob1 = uavPlatform("obstacle_UAV",scene,"ReferenceFrame",'ENU',"InitialPosition",[x_dob1 y_dob1 15],"InitialOrientation",eul2quat([pi/3 0 0]));      % angle orientation ( yaw, pitch, roll)
updateMesh(d_ob1,"fixedwing",{10},color.Red,[0 0 0],eul2quat([0 0 0]))
d_min = [];
%fullscreen()
[ax,plotFrames] = show3D(scene);
xlim([-250 200])
ylim([-150 180])
zlim([0 50])
view([-110 30])
axis equal
setup(scene)
%%
%% Simulation loop
while advance(scene)
    obs = [x_sob1, y_sob1;
       x_sob2, y_sob2;
       x_sob3, y_sob3
       x_dob1, y_dob1 
       ];
    i = i+1;
    [m,l] = read(plat);
    x_pos = m(1);
    y_pos = m(2);
    Vx = V*cos(phi);
    Vy = V*sin(phi);
    path_UAV(i,1) = x_pos;
    path_UAV(i,2) = y_pos;
    path_UAV(i,3) = 15;
    eul = quat2eul(m(10:13));
    theta = eul(1);
    phi_obs = mpf_multi_obs(x_pos,y_pos,theta,obs,v_ob,b,k_rep0,V,x_goal,y_goal);
%     phi_wp = traj_follow(w2,w1,x_pos,y_pos,phi_e,a,c,V,theta);
    phi =  phi_obs;
    com_dif = angdiff(theta,phi);
    roll = atan(V*com_dif/9.81);
    roll_angle(i,1) = i;
    roll_angle(i,2) = roll*180/pi;
    x_pos = x_pos + V*cos(phi);
    y_pos = y_pos + V*sin(phi);
    
    x_dob1 = x_dob1 + v_ob(4,1);
    y_dob1 = y_dob1 + v_ob(4,2);
    d_min(i) = sqrt((x_pos - x_dob1)^2+(y_pos - y_dob1)^2);
    path_obs1(i,1) = x_dob1;
    path_obs1(i,2) = y_dob1;
    path_obs1(i,3) = 15;

    move(plat,[[x_pos y_pos 15], [V.*cos(phi) V.*sin(phi) 0], zeros(1,3), eul2quat([phi 0 roll]), [0 0 0]])
    move(d_ob1,[[x_dob1 y_dob1 15], [v_ob(4,1) v_ob(4,2) 0], zeros(1,3), eul2quat([atan(v_ob(4,2)/v_ob(4,1)) 0 0]), [0 0 0]])

    show3D(scene); 
    view(-30,60)
    hold on
    plot3(path_UAV(:,1),path_UAV(:,2),path_UAV(:,3))
    plot3(path_obs1(:,1),path_obs1(:,2),path_obs1(:,3))

    drawnow update 
    hold off
    
    if(sqrt((x_pos - x_goal)^2+(y_pos - y_goal)^2) < 15)
        break
    end
gif
end
figure
plot(roll_angle(:,1),roll_angle(:,2))
ylabel('Roll angle (deg)')
xlabel('Time (sec)')
% 
% figure
% plot(heading_angle(:,1),heading_angle(:,2))

disp(min(d_min))