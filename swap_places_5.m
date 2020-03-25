function [num_robots, delta_t, tau, robots] = swap_places_5()
    num_robots = 5;
    delta_t = 0.01;
    tau = 2;
    
    robots(1).cur_pos = [cos(2*pi/5);sin(2*pi/5)];
    robots(1).cur_vel = [0;0];
    robots(1).radius = 0.1;
    robots(1).max_speed = 3;
    robots(1).goal_pos = -[cos(2*pi/5);sin(2*pi/5)];
    direction = robots(1).goal_pos-robots(1).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(1).pref_vel = direction*robots(1).max_speed;
    
    robots(2).cur_pos = [cos(2*2*pi/5);sin(2*2*pi/5)];
    robots(2).cur_vel = [0;0];
    robots(2).radius = 0.1;
    robots(2).max_speed = 3;
    robots(2).goal_pos = -[cos(2*2*pi/5);sin(2*2*pi/5)];
    direction = robots(2).goal_pos-robots(2).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(2).pref_vel = direction*robots(2).max_speed;
    
    robots(3).cur_pos = [cos(3*2*pi/5);sin(3*2*pi/5)];
    robots(3).cur_vel = [0;0];
    robots(3).radius = 0.1;
    robots(3).max_speed = 3;
    robots(3).goal_pos = -[cos(3*2*pi/5);sin(3*2*pi/5)];
    direction = robots(3).goal_pos-robots(3).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(3).pref_vel = direction*robots(3).max_speed;
    
    robots(4).cur_pos = [cos(4*2*pi/5);sin(4*2*pi/5)];
    robots(4).cur_vel = [0;0];
    robots(4).radius = 0.1;
    robots(4).max_speed = 3;
    robots(4).goal_pos = -[cos(4*2*pi/5);sin(4*2*pi/5)];
    direction = robots(4).goal_pos-robots(4).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(4).pref_vel = direction*robots(4).max_speed;
    
    robots(5).cur_pos = [cos(0);sin(0)];
    robots(5).cur_vel = [0;0];
    robots(5).radius = 0.1;
    robots(5).max_speed = 3;
    robots(5).goal_pos = -[cos(0);sin(0)];
    direction = robots(5).goal_pos-robots(5).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(5).pref_vel = direction*robots(5).max_speed;
end