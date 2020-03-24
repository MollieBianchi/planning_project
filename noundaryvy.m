function vy = noundaryvy(vx, robot_A, robot_B, delta_t)


vy = ((robot_B.cur_pos(2) - robot_A.cur_pos(2))/delta_t + sqrt(((robot_A.radius + robot_B.radius)/delta_t)^2 - (vx - (robot_B.cur_pos(1) - robot_A.cur_pos(1))/delta_t)^2));

end