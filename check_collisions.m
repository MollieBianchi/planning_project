function num_collisions = check_collisions(poses, num_robots, radius)
    num_collisions = 0;
    for i = 1:num_robots
        for j = i+1:num_robots
             num_collisions = num_collisions + sum((sqrt((poses(:,1,i)-poses(:,1,j)).^2+ (poses(:,2,i)-poses(:,2,j)).^2)) < (2*radius));
        end
    end

end