function g = mpf(r_pos,obs_pos,v_obs,wp_pos,k_att,k_rep,v_UAV)
    n = size(obs_pos,1);
    z_pos = 0;
    theta = zeros(n,1);
    phi = zeros(n,1);
    v_rel = zeros(n,1);
    for i=1:n
        x = r_pos(1);
        y = r_pos(2);
        phi(i) = acos()
        v_rel(i) = [v_obs(i,2)-v_UAV(2);v_obs(i,2)-v_UAV(2)];
        mag_vel = sqrt((v_obs(i,2)-v_UAV(2)).^2+(v_obs(i,2)-v_UAV(2)).^2);
        theta(i) = abs(atan(v_rel(i,2)/v_rel(i,1)));
        z_pos = z_pos + (k_rep.*(mag_vel).*(((cos(theta(i))+2).*((((r_pos(1)-obs_pos(i,1)).*cos(theta(i))+(y-y_ob).*sin(theta2))./50).^2+(((x-x_ob).*sin(theta2)-(y-y_ob).*cos(theta2))./20).^2)+1))).^(-1);
    end
end