function [path_length, smoothness] = get_path_length(poses, num_robots, iterations)
    path_length = 0;
    smoothness = 0;
    for i = 1:num_robots
        
        % path length measure
        for j =2:iterations
            path_length = path_length + sqrt(sum((poses(j-1,:,i) - poses(j,:,i)).^2));
        end
        
        x_diff_1 = diff(poses(:,1,i));
        x_diff_2 = diff(x_diff_1);
        
        y_diff_1 = diff(poses(:,2,i));
        y_diff_2 = diff(y_diff_1);
        
        curvature = (x_diff_1(2:end).*y_diff_2 - y_diff_1(2:end).*x_diff_2)./(x_diff_1(2:end).^2+y_diff_1(2:end).^2).^(3/2);
        
        smoothness = sqrt(sum(diff(curvature).^2));
%         % smoothness measure
%         x_diff = diff(poses(:,1,i));
%         x_diff(abs(x_diff)<0.00001) = 0;
%         y_diff = diff(poses(:,2,i));
%         slopes = y_diff./x_diff;
%         slopes(slopes==Inf) = 10000;
%         slopes(slopes==-Inf) = -10000;
%         smoothness = smoothness + sqrt(sum(diff(diff(slopes)) .^2));
    end

    %smoothness = smoothness/path_length;
end