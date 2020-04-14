function [num_robots, delta_t, tau, robots] = swap_places_3()
    num_robots = 3;
    delta_t = 0.05;
    tau = 0.1;
    
    robots(1).cur_pos = [2*cos(2*pi/num_robots);2*sin(2*pi/num_robots)];
    robots(1).cur_vel = [0;0];
    robots(1).radius = 0.1;
    robots(1).max_speed = 1;
    robots(1).goal_pos = -[2*cos(2*pi/num_robots);2*sin(2*pi/num_robots)];
    direction = robots(1).goal_pos-robots(1).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(1).pref_vel = direction*robots(1).max_speed;
    
    robots(2).cur_pos = [2*cos(2*2*pi/num_robots);2*sin(2*2*pi/num_robots)];
    robots(2).cur_vel = [0;0];
    robots(2).radius = 0.1;
    robots(2).max_speed = 1;
    robots(2).goal_pos = -[2*cos(2*2*pi/num_robots);2*sin(2*2*pi/num_robots)];
    direction = robots(2).goal_pos-robots(2).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(2).pref_vel = direction*robots(2).max_speed;
    
    robots(3).cur_pos = [2*cos(3*2*pi/num_robots);2*sin(3*2*pi/num_robots)];
    robots(3).cur_vel = [0;0];
    robots(3).radius = 0.1;
    robots(3).max_speed = 1;
    robots(3).goal_pos = -[2*cos(3*2*pi/num_robots);2*sin(3*2*pi/num_robots)];
    direction = robots(3).goal_pos-robots(3).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(3).pref_vel = direction*robots(3).max_speed;
    
   
end