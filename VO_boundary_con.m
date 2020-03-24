function [c,ceq] = VO_boundary_con(x)
    c = sum(x(1:2).^2) - sum(x(3:4).^2);
    ceq = sqrt(sum((x(1:2) - x(3:4)).^2)) - x(5);
end