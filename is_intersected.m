function b = is_intersected(parameters, old_pose)
% return 1 if iCreate is on m-line
% return 0 otherwise

    mk = parameters(1);
    mb = parameters(2);

    current_pos = pos_from_ht(old_pose);

    tolerance = 0.1;

    % I.e. whenever we P_y == kP_x + b, we are on m-line.
    if abs(current_pos(2) - mk * current_pos(1) - mb) <= tolerance
        b = 1;
    else
        b = 0;
    end

end