function circle(pos,r,A)
    %pos contains x and y which are the coordinates of the center of the circle
    %r is the radius of the circle
    if A ==1
        color = 'r';
    else
        color = 'b';
    end
    x = pos(1);
    y = pos(2);
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    figure(3)
    hold on
    plot(x+xp,y+yp, 'linewidth', 1, 'color', color);

    xlim([0,4]);
    ylim([0,4]);
    
end