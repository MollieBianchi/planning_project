function [num_robots, delta_t, tau, robots] = swap_places()
    num_robots = 2;
    delta_t = 0.05;
    tau = 0.5;
    robots(1).cur_pos = [0;-2];
    robots(1).cur_vel = [0;0];
    robots(1).radius = 0.1;
    robots(1).max_speed = 1;
    robots(1).goal_pos = [0;2];
    robots(1).pref_vel = max(-robots(1).max_speed+0.1, min(robots(1).max_speed-0.1, (robots(1).goal_pos-robots(1).cur_pos)/delta_t));
    
    robots(2).cur_pos = [0;2];
    robots(2).cur_vel = [0;0];
    robots(2).radius = 0.1;
    robots(2).max_speed = 1;
    robots(2).goal_pos = [0;-2];
    robots(2).pref_vel = max(-robots(2).max_speed+0.1,min(robots(2).max_speed-0.1, (robots(2).goal_pos-robots(2).cur_pos)/delta_t));
end