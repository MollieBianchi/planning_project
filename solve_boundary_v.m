function [u,n] = solve_boundary_v(robot_id, robot_A, robot_B, tau)
    
    vel_target = robot_A.cur_vel - robot_B.cur_vel;
    radius = (robot_A.radius + robot_B.radius)/tau;

    center = (robot_B.cur_pos - robot_A.cur_pos)/tau;

    % solve for slope of two lines tanget to this circle
    A = 4*radius^2-4*center(1)^2;
    B = 8*center(1)*center(2);
    C = 4*radius^2 - 4*center(2)^2;

    k = roots([A,B,C]);
    
    % solve for interceptions with circle
    if length(k) == 2
        A = 1+k(1)^2;
        B = -2*center(1) - 2*center(2)*k(1);
        C = center(1)^2 + center(2)^2 - radius^2;

        x1 = roots([A,B,C]);
        y1 = k(1)*x1(1);

        A = 1+k(2)^2;
        B = -2*center(1) - 2*center(2)*k(2);
        C = center(1)^2 + center(2)^2 - radius^2;

        x2 = roots([A,B,C]);
        y2 = k(2)*x2(1);
        
        upperbound = real([x1(1), y1]);    
        lowerbound = real([x2(1), y2]);
    else
        % one line is vertical
        A = 1+k^2;
        B = -2*center(1) - 2*center(2)*k;
        C = center(1)^2 + center(2)^2 - radius^2;

        x = roots([A,B,C]);
        y = k*x(1);
        
        upperbound = real([0,0]);
        lowerbound = real([x(1),y]);
       
        k = [inf, k(1)];
    end
    
              
    options = optimoptions('quadprog','Display','off');
     
    % solve quad prog for upper line
    H = eye(2);
    f = -vel_target'*eye(2);
    if upperbound(1) == 0
        % vertical line
        Aeq = [1, 0];
    else
        Aeq = [-k(1), 1];
    end
    beq = [0];
    
    v_upper = quadprog(H,f,[],[],Aeq,beq,[],[],[0;0],options);
    
    % solve quad prog for lower line
    if lowerbound(1) == 0
        % vertical line
        Aeq = [1, 0];
    else
        Aeq = [-k(2), 1];
    end
    beq = [0];
    
    v_lower = quadprog(H,f,[],[],Aeq,beq,[],[],[0;0],options);
    
    % solve fmincon for circle
    fun = @(x)sqrt(sum((x(1:2) - vel_target).^2));

    ub = [20,20,center(1),center(2),radius];
    lb = [-20,-20,center(1),center(2),radius];
    
    Aeq = [0,0,1,0,0;   %p_B(1) - p_A(1)
           0,0,0,1,0;   %p_B(2) - p_A(2)
           0,0,0,0,1];  %r_A + r_B    
       
    beq = [ center(1);
            center(2);
            radius];

    nonlcon = @VO_boundary_con;

    x0 = [ (center(1) + radius/sqrt(2) )/tau;
           (center(2) + radius/sqrt(2) )/tau;
           center(1);
           center(2);
           radius];
       
    options = optimoptions('fmincon', 'Display','off');
    x = fmincon(fun,x0,[],[],Aeq,beq,lb,ub,nonlcon,options);

    v_circle = x(1:2);
    
    
    % find which of the three points are closest
    distances = zeros(1,3);    
    
    % check if closest point on lines is beyond circle
    if sum(v_upper.^2) < sum(upperbound.^2)
        distances(1) = inf;
    elseif ~all(sign(v_upper) == sign(upperbound))
        % check if point has crossed to a different quadrant
        distances(1) = inf;
    else
        distances(1) = sqrt(sum((v_upper - vel_target).^2));
    end
    
    if sum(v_lower.^2) < sum(lowerbound.^2)
        distances(2) = inf;
    elseif ~all(sign(v_lower) == sign(lowerbound))
        distances(2) = inf;
    else
        distances(2) = sqrt(sum((v_lower - vel_target).^2));
    end
    
    if sum(v_circle.^2) > sum(upperbound.^2)
        distances(3) = inf;
    else
        distances(3) = sqrt(sum((v_circle - vel_target).^2));
    end
    
    [~, idx] = min(distances);
    
    if idx == 1
        v = v_upper;
        
        % find normal using slope of upperbound line
        
        % if horizontal line
        if k(1) == 0
            n = [0;1];        
        % if vertical line
        elseif k(1) == Inf
            n = [1;0];
        else
            n = [1;-1/k(1)];            
        end

        % normalize
        n = n/sqrt(sum(n.^2));
        
        % vector pointing towards center line
        c_v = center - v_upper;
       
        % check that its outward normal otherwise flip direction
        if c_v'*n > 0
            n = -n;
        end
               
        
    elseif idx ==2
        v = v_lower;
        
        % find normal using slope of lowerbound line
        if k(2) == 0
            n = [0;1];        
        % if vertical line
        elseif k(2) == Inf
            n = [1;0];
        else
            n = [1;-1/k(2)];            
        end
        
        % normalize
        n = n/sqrt(sum(n.^2));
        
        % vector pointing towards center line
        c_v = center - v_lower;
       
        % check that its outward normal otherwise flip direction
        if c_v'*n > 0
            n = -n;
        end
        
    else
        v = v_circle;
        
        n = v_circle - center;
    end
    
    % solve for u
    u = v - (vel_target);
    
    % plotting for debugging
    figure(robot_id)
    clf
    hold on
    plot(vel_target(1), vel_target(2), 'g*')
    plot(v_upper(1), v_upper(2), 'bo')
    plot(v_lower(1), v_lower(2), 'bo')
    plot(v_circle(1), v_circle(2), 'bo')
    plot(v(1), v(2), 'b*')
    plot(upperbound(1),upperbound(2),'ro')
    plot(lowerbound(1),lowerbound(2),'ro')
    plot(center(1),center(2),'ro')
    
    plot([0, center(1),center(1)*100], [0, center(2),center(2)*100], 'r:')
    plot([0, upperbound(1),upperbound(1)*100], [0, upperbound(2), upperbound(2)*100], 'r--')
    plot([0, lowerbound(1),lowerbound(1)*100], [0, lowerbound(2), lowerbound(2)*100], 'r--')
    
    x = center(1);
    y = center(2);
    ang=0:0.01:2*pi; 
    xp=radius*cos(ang);
    yp=radius*sin(ang);
    plot(x+xp,y+yp, 'r--');

    x = center(1)*tau;
    y = center(2)*tau;
    ang=0:0.01:2*pi; 
    xp=radius*tau*cos(ang);
    yp=radius*tau*sin(ang);
    plot(x+xp,y+yp, 'r--');
    
    xlim([-4,4]);
    ylim([-4,4]);
end





%     % Check if vel_target is inside the VO or outside
%     
%     if sqrt(sum(vel_target.^2)) < sqrt(sum(center.^2))
%         % check if inside circle
%         if sqrt(sum((vel_target - center).^2)) < radius
%             % inside of VO
%             inside_VO = 1;
%         else
%             % outisde of VO
%             inside_VO = 0;
%         end
%     else
%         % check if inbetween upper bound and lower bound        
%         angle = wrapTo2Pi(atan2(vel_target(2), vel_target(1)));
%         angle_upper = wrapTo2Pi(atan2(upperbound(2), upperbound(1)));
%         angle_lower = wrapTo2Pi(atan2(lowerbound(2), lowerbound(1)));
%         
%         % check which bound is on the bottom
%         if wrapTo2Pi(angle_upper - angle_lower) < wrapTo2Pi(angle_lower - angle_upper)
%             % angle lower is on the bottom
%             if angle > angle_lower && angle < angle_upper                
%                 % inside of VO
%                 inside_VO = 1;
%             else
%                 % outside of VO
%                 inside_VO = 0;
%             end
%         else
%             % angle upper is on the bottom
%             if angle > angle_upper && angle < angle_lower               
%                 % inside of VO
%                 inside_VO = 1;
%             else
%                 % outside of VO
%                 inside_VO = 0;
%             end
%         end
%     end
%     
%     % outward normal 
%     if inside_VO == 1
%         n = u/sqrt(sum(u.^2));
%     else
%         n = -u/sqrt(sum(u.^2));
%     end