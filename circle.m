function circle(pos,r,A)
    %pos contains x and y which are the coordinates of the center of the circle
    %r is the radius of the circle
    if A ==1
        color = 'r';
    elseif A == 2
        color = 'g';
    elseif A == 3
        color = 'm';
    elseif A == 4
        color = 'c';
    else
        color = 'b';
    end
    x = pos(1);
    y = pos(2);
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    figure(6)
    hold on
    plot(x+xp,y+yp, 'linewidth', 1, 'color', color);

    xlim([-1.5,1.5]);
    ylim([-1.5,1.5]);
    
end