function circle(pos,r,A)
    %pos contains x and y which are the coordinates of the center of the circle
    %r is the radius of the circle
    if A == 1 || A == 7
        color = 'r';
    elseif A == 2 || A == 8
        color = 'g';
    elseif A == 3 || A == 9
        color = 'm';
    elseif A == 4 || A == 10
        color = 'c';
    elseif A == 5 || A == 11
        color = 'Orange';
    elseif A == 6 || A == 12
        color = 'b';
    end
    x = pos(1);
    y = pos(2);
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    figure(1)
    hold on
    plot(x+xp,y+yp, 'linewidth', 1, 'color', color);

    xlim([-3,3]);
    ylim([-3,3]);
    
end