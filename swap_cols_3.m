function [num_robots, delta_t, tau, robots] = swap_cols_3()
    num_robots = 12;
    delta_t = 0.05;
    tau = 0.1;
    
    robots(1).cur_pos = [-2;2];
    robots(1).cur_vel = [0;0];
    robots(1).radius = 0.1;
    robots(1).max_speed = 1;
    robots(1).goal_pos = [2;-2];
    direction = robots(1).goal_pos-robots(1).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(1).pref_vel = direction*robots(1).max_speed;
    
    robots(2).cur_pos = [-2;1];
    robots(2).cur_vel = [0;0];
    robots(2).radius = 0.1;
    robots(2).max_speed = 1;
    robots(2).goal_pos = [2;-1];
    direction = robots(2).goal_pos-robots(2).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(2).pref_vel = direction*robots(2).max_speed;
    
    robots(3).cur_pos = [-2;-1];
    robots(3).cur_vel = [0;0];
    robots(3).radius = 0.1;
    robots(3).max_speed = 1;
    robots(3).goal_pos = [2;1];
    direction = robots(3).goal_pos-robots(3).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(3).pref_vel = direction*robots(3).max_speed;
    
    robots(4).cur_pos = [-2;-2];
    robots(4).cur_vel = [0;0];
    robots(4).radius = 0.1;
    robots(4).max_speed = 1;
    robots(4).goal_pos = [2;2];
    direction = robots(4).goal_pos-robots(4).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(4).pref_vel = direction*robots(4).max_speed;
    
    robots(5).cur_pos = [0;2];
    robots(5).cur_vel = [0;0];
    robots(5).radius = 0.1;
    robots(5).max_speed = 1;
    robots(5).goal_pos = [0;-1];
    direction = robots(5).goal_pos-robots(5).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(5).pref_vel = direction*robots(5).max_speed;
    
    robots(6).cur_pos = [0;1];
    robots(6).cur_vel = [0;0];
    robots(6).radius = 0.1;
    robots(6).max_speed = 1;
    robots(6).goal_pos = [0;-2];
    direction = robots(6).goal_pos-robots(6).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(6).pref_vel = direction*robots(6).max_speed;
    
    robots(7).cur_pos = [0;-2];
    robots(7).cur_vel = [0;0];
    robots(7).radius = 0.1;
    robots(7).max_speed = 1;
    robots(7).goal_pos = [0;1];
    direction = robots(7).goal_pos-robots(7).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(7).pref_vel = direction*robots(7).max_speed;
    
    robots(8).cur_pos = [0;-1];
    robots(8).cur_vel = [0;0];
    robots(8).radius = 0.1;
    robots(8).max_speed = 1;
    robots(8).goal_pos = [0;2];
    direction = robots(8).goal_pos-robots(8).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(8).pref_vel = direction*robots(8).max_speed;
    
    robots(9).cur_pos = [2;2];
    robots(9).cur_vel = [0;0];
    robots(9).radius = 0.1;
    robots(9).max_speed = 1;
    robots(9).goal_pos = [-2;-2];
    direction = robots(9).goal_pos-robots(9).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(9).pref_vel = direction*robots(9).max_speed;
    
    robots(10).cur_pos = [2;1];
    robots(10).cur_vel = [0;0];
    robots(10).radius = 0.1;
    robots(10).max_speed = 1;
    robots(10).goal_pos = [-2;-1];
    direction = robots(10).goal_pos-robots(10).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(10).pref_vel = direction*robots(10).max_speed;
    
    robots(11).cur_pos = [2;-1];
    robots(11).cur_vel = [0;0];
    robots(11).radius = 0.1;
    robots(11).max_speed = 1;
    robots(11).goal_pos = [-2;1];
    direction = robots(11).goal_pos-robots(11).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(11).pref_vel = direction*robots(11).max_speed;
    
    robots(12).cur_pos = [2;-2];
    robots(12).cur_vel = [0;0];
    robots(12).radius = 0.1;
    robots(12).max_speed = 1;
    robots(12).goal_pos = [-2;2];
    direction = robots(12).goal_pos-robots(12).cur_pos;
    direction = direction/sqrt(sum(direction.^2));
    robots(12).pref_vel = direction*robots(12).max_speed;
end