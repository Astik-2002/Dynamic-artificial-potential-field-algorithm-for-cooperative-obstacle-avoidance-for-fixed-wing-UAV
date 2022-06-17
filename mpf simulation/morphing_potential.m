function mpf = morphing_potential(x,y,obs,v_rel,a,b,k_rep0,x_goal,y_goal,k_att)
mpf = 0;
for i = 1:length(obs)
    theta = atan(abs(v_rel(i,2)/v_rel(i,1)));
    X = x - obs(i,1);
    Y = y - obs(i,2);
    mpf = mpf + k_rep0./(((X.*cos(theta)+Y.*sin(theta))./a).^2+((X.*sin(theta)-Y.*cos(theta))./b).^2+1);
end
mpf = mpf + k_att.*((x-x_goal).^2+(y-y_goal).^2);
end