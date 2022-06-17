function num_collision = collision_report(x_pos,y_pos,obs)
num_collision = 0;    
for i = 1:length(obs)
        d = sqrt((obs(i,1)-x_pos)^2+(obs(i,2)-y_pos)^2);
        if( d <= 10)
            num_collision = num_collision + 1;
        end
end
end