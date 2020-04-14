
% intialize the problem
[num_robots, delta_t, tau, robots] = swap_places();

poses = zeros(2000,2,num_robots);

for  A = 1:num_robots
    robot = robots(A);
    % save robot positions
    poses(1,:,A) = robot.cur_pos;
end

for i = 2:2000
    
    new_velocities = zeros(2,num_robots);
    
    for A = 1:num_robots
        robot_A = robots(A);
        half_planes = zeros(2,num_robots-1);
        normals = zeros(2,num_robots-1);
        c = 0;
%         for B = 1:num_robots
%             if B ~= A
%                 robot_B = robots(B);
%                 
%                 % add a check in case the robot is too far
%                 %if sqrt(sum((robot_B.cur_pos - robot_A.cur_pos).^2)) - robot_A.radius - robot_B.radius <= (robot_B.max_speed+robot_A.max_speed)*tau
%                     % compute half plane                
%                     % solve for v on boundary
%                    
%                     [u,n] = solve_boundary_v(A, robot_A, robot_B, tau);  
%                    
%                     c = c + 1;
%                     half_planes(:,c) = u;  
%                     normals(:,c) = n;
%                 %end
%             end            
%         end
        
        % select a new velocity        
        H = eye(2);
        f = -robot_A.pref_vel'*eye(2);
        A_con = [];
        b_con = [];
        
        for con = 1:c
            u = half_planes(:,con);
            n = normals(:,con);
            
            A_con(con, 1:2) = -n';
            b_con(con) = -(robot_A.cur_vel + 1/2*u)'*n;
            
            %plot_velocity(robot_A.pref_vel, robot_A.cur_vel, u, n, A);
        end
        UB = [robot_A.max_speed, robot_A.max_speed];
        LB = [-robot_A.max_speed, -robot_A.max_speed];
        
        options = optimoptions('quadprog','Display','off');
        v = quadprog(H, f, A_con, b_con, [],[],[],[], [0;0], options);
        
%         if A == 1
%             figure(A);
%             hold on
%             plot(v(1), v(2), 'm*');
%         end
        
        new_velocities(1:2,A) = v;
        %new_velocities(1:2,A) = robot_A.pref_vel;
    end
    
    all_robots_at_goal = true;
    
    for A = 1:num_robots
        robot = robots(A);
        
        % update velocities
        robot.cur_vel = new_velocities(1:2,A); %+ ; %%%%%%% INSERT NOISE HERE
        
        % update positions
        robot.cur_pos = robot.cur_pos + robot.cur_vel*delta_t;
        
        % check if at goal
        if sum(abs(robot.cur_pos - robot.goal_pos) > [0.01; 0.01]) ~= 0
            all_robots_at_goal = false;
        end
        
        % update preferred velocities
        direction = robot.goal_pos-robot.cur_pos;
        direction = direction/sqrt(sum(direction.^2));
        
        robot.pref_vel =  get_v_pref(robot, robots, pi, 0, A, delta_t);  %%%%% CHANGE RADIUS VARIABLE HERE
        
        % update robots
        robots(A) = robot;
        
        % save robot positions
        poses(i,:,A) = robot.cur_pos;
        
        % update plot
        %circle(robot.cur_pos, robot.radius,A);
    end
    
    % check if all robots are at goal and then break
    if all_robots_at_goal
        break
    end
end

'Collisions'
check_collisions(poses(1:i,:,:), num_robots, robots(1).radius)

"Total Path Length and Smoothness"
[length, smoothness] = get_path_length(poses(1:i,:,:), num_robots, i)

"Smoothness/Length"
smoothness/length

'Iterations'
i

figure(6);
hold on
plot(poses(1:i,1,1), poses(1:i,2,1), 'r')
plot(poses(1:i,1,2), poses(1:i,2,2), 'g')


% plot(poses(1:i,1,3), poses(1:i,2,3), 'c')
% plot(poses(1:i,1,4), poses(1:i,2,4), 'b')
% plot(poses(1:i,1,5), poses(1:i,2,5), 'color', [0.5 0 1])
% plot(poses(1:i,1,6), poses(1:i,2,6), 'c')
% plot(poses(1:i,1,7), poses(1:i,2,7), 'm')
% plot(poses(1:i,1,8), poses(1:i,2,8), 'g')
% plot(poses(1:i,1,9), poses(1:i,2,9), 'r')
% plot(poses(1:i,1,10), poses(1:i,2,10), 'm')
% plot(poses(1:i,1,11), poses(1:i,2,11), 'color', [0.5 0 1])
% plot(poses(1:i,1,12), poses(1:i,2,12), 'b')
% 
% figure(5);
% hold on
% plot(1:i,sqrt((poses(:,1,1)-poses(:,1,2)).^2+ (poses(:,2,1)-poses(:,2,2)).^2), 'g')
% plot(1:i,repmat(2*robots(1).radius, 1, i));