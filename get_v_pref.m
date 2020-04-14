function v_pref = get_v_pref(robot, robots, phi, radius, A, dt)

    % velocity directly towards goal
    direction = robot.goal_pos-robot.cur_pos;
    
    % if already at goal return speed of zero
    if sum(abs(direction) > [0.01; 0.01]) == 0
        v_goal = [0;0];
        v_left = [0;0];
        v_pref = [0;0];
        
    else
        v_goal = direction/dt;

        robot_heading = atan2(v_goal(2),v_goal(1));
        
        v_left = [0;0];
        alpha = 0;
        % check all other robots
        for i = 1:length(robots)
            if i ~= A
                range = robots(i).cur_pos - robot.cur_pos ;
                diff_ang = atan2(range(2), range(1));
                range = sqrt(sum(range.^2));

                bearing = robot_heading-diff_ang;
                if abs(bearing) > 0.1
                    bearing = wrapTo2Pi(bearing);
                end

                % if robot in detection radius
                if range < radius && bearing < phi
                    % turn left a bit
                    v_left = [ v_goal(1)*cos(pi/2) - v_goal(2)*sin(pi/2);
                               v_goal(1)*sin(pi/2) + v_goal(2)*cos(pi/2)];
                    alpha = 0.3*(radius-range);
                    
                    break
                end
            end  
        end
        
        v_pref = v_goal + alpha*v_left;
    end
    
    % limit v_pref by max_speed
    if sqrt(sum(v_pref.^2)) > robot.max_speed
        v_pref = v_pref/sqrt(sum(v_pref.^2))*robot.max_speed;
    end
%     if true%A == 3
%         figure(A+1)
%         clf
%         hold on
%         plot([0,v_pref(1)],[0,v_pref(2)])
%         plot([0,v_goal(1)],[0,v_goal(2)])
%         plot([0,v_left(1)],[0,v_left(2)])
%         xlim([-3,3]);
%         ylim([-3,3]);
%         legend('v_pref', 'v_goal', 'v_left')
%         %v_pref
%     end
end