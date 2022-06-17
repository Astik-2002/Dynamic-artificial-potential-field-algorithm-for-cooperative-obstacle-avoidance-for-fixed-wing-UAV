function vel_UAV = mpf_multi_obs(x_pos,y_pos,obs,v_rel,a,b,k_rep0,x_goal,y_goal,k_att)
vx = 0;
vy = 0;
for i = 1:length(obs)
    
    theta = atan(abs(v_rel(i,2)/v_rel(i,1)));
    
    X = x_pos - obs(i,1);
    Y = y_pos - obs(i,2);
    
    z = ((X*cos(theta)+Y*sin(theta))/a)^2+((X*sin(theta)-Y*cos(theta))./b).^2+1;

    phi = (X*cos(theta)+Y*sin(theta))/(sqrt(X^2+Y^2));
    phi2 = acos(phi)+(pi/3);
    r = cos(phi2);
    k_rep = (r^2*(k_rep0-1)+1)*(sqrt((v_rel(i,1))^2+(v_rel(i,2))^2));

    % if(phi < 0.5)
    %k_rep = k_rep*();
    vx = vx + 2*k_rep*(((Y*sin(theta))/b^2)-((X*cos(theta))/a^2))/(z^2);
    vy = vy - 2*k_rep*(((Y*cos(theta))/b^2)+((X*sin(theta))/a^2))/(z^2);
    % else
    %     vx = 0;
    %     vy = 0;
    % end
end
vx = vx + k_att*(x_pos - x_goal);
vy = vy + k_att*(y_pos - y_goal);


vel_UAV = [vx;vy];
end