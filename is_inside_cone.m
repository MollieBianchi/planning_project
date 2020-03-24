function inside_cone = is_inside_cone(test_point, upperbound, lowerbound)
    angle_test = wrapTo2Pi(atan2(test_point(2), test_point(1)));
        
    angle_upper = wrapTo2Pi(atan2(upperbound(2), upperbound(1)));
    angle_lower = wrapTo2Pi(atan2(lowerbound(2), lowerbound(1)));

    % try going up and to the right
    % if this point is inside
    % go down and left

    if wrapTo2Pi(angle_upper - angle_lower) < wrapTo2Pi(angle_lower - angle_upper)
        % angle lower is on the bottom
        if angle_test > angle_lower && angle_test < angle_upper                
            % inside of VO
            inside_cone = 1;
        else
            % outside of VO
            inside_cone = 0;
        end
    else
        % angle upper is on the bottom
        if angle_test > angle_upper && angle_test < angle_lower               
            % inside of VO
           inside_cone = 1;
        else
            % outside of VO
            inside_cone = 0;
        end
    end
end