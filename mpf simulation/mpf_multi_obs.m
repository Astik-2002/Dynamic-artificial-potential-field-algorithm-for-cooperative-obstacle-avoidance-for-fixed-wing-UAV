function yaw = mpf_multi_obs(x_pos,y_pos,theta,obs,v_ob,b,k_rep0,V,x_goal,y_goal)
    n = length(obs);
    a = 2.*b;
    vx = 0;
    vy = 0;
    v_UAV = [V*cos(theta);V*sin(theta)];
    for i = 1:n
        v_rel(1) = v_ob(i,1) - v_UAV(1);
        v_rel(2) = v_ob(i,2) - v_UAV(2);
        phi = atan(abs(v_rel(2)/v_rel(1)));
        obs(i,1) = obs(i,1) + v_ob(i,1);
        obs(i,2) = obs(i,2) + v_ob(i,2);
        x = x_pos - obs(i,1);
        y = y_pos - obs(i,2);
        eta = (x*v_rel(1)+y*v_rel(2))/(sqrt((v_rel(1))^2+(v_rel(2))^2)*sqrt((x)^2+(y)^2));
        k_rep0 = k_rep0*(sin(pi*eta/2)+1);
        a(i) = b(i)*(1.73 + eta^2);
        z = 0.34*((x*cos(phi)+y*sin(phi))/a(i))^2+((x*sin(phi)-y*cos(phi))/b(i))^2+1;
        vx = vx + 2*k_rep0*(((x*sin(phi))/b(i)^2)-((x*cos(phi))/a(i)^2))/(z^2);
        vy = vy - 2*k_rep0*(((y*cos(phi))/b(i)^2)+((x*sin(phi))/a(i)^2))/(z^2);
    end

v = [vx;vy] + 0.008.*[x_goal - x_pos; y_goal - y_pos];
yaw = acos(v(1)/sqrt(v(1)^2+v(2)^2));
end
