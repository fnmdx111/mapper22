function b = is_intersected(parameters, old_pose)
% return 1 if iCreate is on m-line
% return 0 otherwise

    % We are solving a linear equation here
    % the point we start and the goal point forms a line,
    % in this particular case, it's y = 0,
    % So it satisfies that whenever the y value of our
    % coordinate is y (of course you don't do `equal float`),
    % we are on m-line.

    global mw_g_circumnavigate_tolerance

    mk = parameters(1);
    mb = parameters(2);

    current_pos = pos_from_ht(old_pose);

    tolerance = mw_g_circumnavigate_tolerance;

    if abs(current_pos(2) - mk * current_pos(1) - mb) <= tolerance
        b = 1;
    else
        b = 0;
    end

end