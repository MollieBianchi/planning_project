function plot_velocity(pref_vel, cur_vel, u, v, n, A)
    %pos contains x and y which are the coordinates of the center of the circle
    %r is the radius of the circle
  
    figure(4)
    clf
    hold on
    
    plot(pref_vel(1), pref_vel(2), 'bo')    
    plot([cur_vel(1),cur_vel(1)+u(1)/2], [cur_vel(2),cur_vel(2)+u(2)/2], '-o', 'color', 'r');
    plot(v(1), v(2), 'm*')
    quiver(cur_vel(1)+u(1)/2, cur_vel(2)+u(2)/2, n(1), n(2), 0, 'MaxHeadSize', 1, 'LineWidth', 3, 'Color', 'b');
    plot(cur_vel(1), cur_vel(2), 'go')
    xlim([-3.5, 3.5])
    ylim([-3.5, 3.5])
end