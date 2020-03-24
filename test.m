

radius = 1;

center = [1,2];
    
A = 4*radius^2-4*center(1)^2;
B = 8*center(1)*center(2);
C = 4*radius^2 - 4*center(2)^2;

k = roots([A,B,C])

figure(1)
hold on
plot(center(1),center(2), 'o')
x = center(1);
y = center(2);
ang=0:0.01:2*pi; 
xp=radius*cos(ang);
yp=radius*sin(ang);
plot(x+xp,y+yp, 'r--');
plot([-10,0,10], [-10*k(1), 0, 10*k(1)], ':')
plot([-10,0,10], [-10*k(2), 0, 10*k(2)], ':')

xlim([-1,1])
ylim([-1,1])

% 
% figure(1)
% hold on
% for x = -1:0.1:1
%   
%     hold on
%     y = sqrt(1-x^2);
%     
%     upperbound = [x + 0.2*cos(atan2(x,y));
%                   y - 0.2*sin(atan2(x,y))];
%     
%     lowerbound = [x - 0.2*cos(atan2(x,y));
%                   y + 0.2*sin(atan2(x,y))];
%        
%     plot(x, atan2(y,x), 'o')
% %     plot([x,x*10],[y,y*10],'ro-')
% %     plot([upperbound(1),upperbound(1)*10],[upperbound(2), upperbound(2)*10],'bo-')
% %     plot([lowerbound(1),lowerbound(1)*10],[lowerbound(2), lowerbound(2)*10],'go-')
%     xlim([-10,10])
%     ylim([-10,10])
% end
% for x = -1:0.1:1
%   
%     hold on
%     y = -sqrt(1-x^2);
%     
%     upperbound = [x + 0.2*cos(atan2(x,y));
%                   y - 0.2*sin(atan2(x,y))];
%     
%     lowerbound = [x - 0.2*cos(atan2(x,y));
%                   y + 0.2*sin(atan2(x,y))];
%        
%     plot(x, atan2(y,x), 'o')
% %     plot([x,x*10],[y,y*10],'ro-')
% %     plot([upperbound(1),upperbound(1)*10],[upperbound(2), upperbound(2)*10],'bo-')
% %     plot([lowerbound(1),lowerbound(1)*10],[lowerbound(2), lowerbound(2)*10],'go-')
%     xlim([-10,10])
%     ylim([-10,10])
% end
% 
% 
% figure(1)
% hold on
% x = 0.0995
% y = -0.0820
% 
% upperbound = [x + 0.1*cos(atan2(x,y));
%                   y - 0.1*sin(atan2(x,y))];
% lowerbound = [x - 0.1*cos(atan2(x,y));
%                   y + 0.1*sin(atan2(x,y))];
%               
% plot([0,x,10*x],[0,y,10*y],'ro-')
% plot([0,upperbound(1)],[0,upperbound(2)],'bo-')
% plot([0,lowerbound(1)],[0,lowerbound(2)],'go-')

% center
% -0.1281
% 0.0815
% 
% upperbound
% -0.0744
% 0.1659
%    
% lowerbound
%  -0.1818
 -0.0028


% delta_t = 2;
% 
% robots(1).cur_pos = [1;2];
% robots(1).cur_vel = [0;0];
% robots(1).radius = 0.1;
% robots(1).max_speed = 3;
% robots(1).goal_pos = [3;2];
% robots(1).pref_vel = max(-robots(1).max_speed+0.1, min(robots(1).max_speed-0.1, (robots(1).goal_pos-robots(1).cur_pos)/delta_t));
% 
% robots(2).cur_pos = [3;2];
% robots(2).cur_vel = [0;0];
% robots(2).radius = 0.1;
% robots(2).max_speed = 3;
% robots(2).goal_pos = [1;2];
% robots(2).pref_vel = max(-robots(2).max_speed+0.1,min(robots(2).max_speed-0.1, (robots(2).goal_pos-robots(2).cur_pos)/delta_t));
% 
% 
% 
% 
% vx11 = 1.8:0.01:2.2;
% vy11 = [];
% for i = 1:41
%     vy11(i) = noundaryvy(vx11(i), robots(1), robots(2), 1);
% end
% 
% vx1 = 0.9:0.01:1.1;
% vy1 = [];
% for i = 1:21
%     vy1(i) = noundaryvy(vx(i), robots(1), robots(2), delta_t);
% end
% 
% vx2 = -1.1:0.01:-0.9
% vy2 = [];
% for i = 1:21
%     vy2(i) = noundaryvy(vx2(i), robots(2), robots(1), delta_t);
% end
% 
% radius = robot_A.radius+ robot_B.radius;
% 
% figure(1)
% hold on
% % center
% center1 = robots(2).cur_pos - robots(1).cur_pos;
% plot(center1(1), center1(2), 'or')
% % upper bound
% upperbound = [center1(1) + radius*cos(atan2(center1(1),center1(2)));
%               center1(2) - radius*sin(atan2(center1(1),center1(2)))];
% plot(upperbound(1), upperbound(2), '*r')
% % lower bound
% lowerbound = [center1(1) - radius*cos(atan2(center1(1),center1(2)));
%               center1(2) + radius*sin(atan2(center1(1),center1(2)))];
% plot(lowerbound(1), lowerbound(2), '*r')
% % plot center line
% plot(0:10, center1(2)/center1(1)*(0:10), 'r')
% % plot upper line
% plot(0:10, upperbound(2)/upperbound(1)*(0:10), 'r--')
% % plot lower line
% plot(0:10, lowerbound(2)/lowerbound(1)*(0:10), 'r--')
% % truncated circle
% plot(vx1,vy1, 'r');
% plot(vx1,-vy1, 'r');
% % truncated circle
% plot(vx11,vy11, 'r');
% plot(vx11,-vy11, 'r');
% 
% plot(robots(1).cur_pos(1) - robots(2).cur_pos(1), robots(1).cur_pos(2) - robots(2).cur_pos(2), '*b')
% plot(vx2,vy2, 'b');
% plot(vx2,-vy2, 'b');
% xlim([-5,5]);
% ylim([-5,5]);
% 
% 


% function v = solve_boundary_V(robot_A, robot_B, T)
%     fun = @(x)sum(abs((x(1:2) - (robot_A.cur_vel - robot_B.cur_vel))));
% 
%     ub = [3,3,T];
%     lb = [-3,-3,0];
% 
%     Aeq = [0,0,0,1,0,0;   %p_B(1) - p_A(1)
%            0,0,0,0,1,0;   %p_B(2) - p_A(2)
%            0,0,0,0,0,1]; %r_A + r_B            
%     beq = [ robot_B.cur_pos(1) - robot_A.cur_pos(1);
%              robot_B.cur_pos(2) - robot_A.cur_pos(1);
%              robot_A.radius + robot_B.radius];
% 
%     nonlcon = @VO_boundary_con;
% 
%     x0 = [ ;
%            ;
%            T;
%            robot_B.cur_pos(1) - robot_A.cur_pos(1);
%            robot_B.cur_pos(2) - robot_A.cur_pos(1);
%            robot_A.radius + robot_B.radius];
%     
%     x = fmincon(fun,x0,[],[],Aeq,beq,lb,ub,nonlcon);
% 
%     v = x(1:2);
% 
% end