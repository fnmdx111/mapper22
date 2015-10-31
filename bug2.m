function end_pose = bug2(r, start_pose, goalx, goaly, work_mode)
% We use bug2 to traverse all the endpoints calculated in a
% specific order, hoping the trajectory of the robot will cover
% the environment as much as possible.

    % work_mode: 0 - find boundary: robot sets its destination
    %                to (infinity, 0) and try to map out the
    %                outmost boundary of the current environment
    %
    %            1 - traverse: the classic Bug2 algorithm

    global circumnavigate_ok

    global gw_g_bug2_tolerance
    gw_g_bug2_tolerance = 0.19;

    goal = [goalx goaly];
    % We have a BUG here, if we use a 2D vector as one of the
    % arguments of `bug2`, it will discard the y value of this
    % vector, i.e. the vector [x, y] after being passed in becomes
    % [x, x].

    pose = start_pose;

    global boundary_done
    global BOUNDARY

    if work_mode == 0
        boundary_done = false;
    end

    global VISITED

    CALIBRATE_TOLERANCE = 0.06;

    while true
        pose = turn_towards_dest(r, goal, pose);

        OS = pos_from_ht(pose);
        SP = (goal - OS)';
        orientation = [pose(1, 1) pose(2, 1)]';
        sin_theta = sqrt(1 - dot(orientation, SP / norm(SP)) ^ 2);

        % We do this every 3 steps:
        %
        %                           point where the robot currently
        %              * (R_x, R_y) is at
        %             /|---
        %            / | ^
        %           /  | |
        %          /   | d?
        %   rv -> ^    | |
        %        /)?   | v
        %       *------+---------* (P_x, P_y) destination
        %       (S_x, S_y)
        %       point where we start bug2
        %
        %  rv = [pose(1, 1) pose(2, 1)]'
        %  sin? = sqrt(1 - (rv . (SP / ||SP||))^2)
        %  d? = ||RS||sin?
        %  If d? > epsilon, calibrate the orientation!

        step_count = 0;

        bump = bump_test(r);
        while bump == NO_BUMP
            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);
            display(norm(pose(1:2, 3) - goal'))

            if step_count > 2
                RS_length = norm(OS - pos_from_ht(pose));
                y = RS_length * sin_theta;
                display(y)

                if abs(y) > CALIBRATE_TOLERANCE
                    pose = turn_towards_dest(r, goal, pose);
                end

                step_count = 0;
            end
            step_count = step_count + 1;

            if work_mode == 1
                VISITED(end+1, :) = pos_from_ht(pose);
            end

            if norm(pose(1:2, 3) - goal') < gw_g_bug2_tolerance
                display('SUCCEED')
                SetFwdVelRadiusRoomba(r, 0, inf);

                circumnavigate_ok = 1;
                end_pose = pose;

                return
            end

            bump = bump_test(r);
        end
    
        if work_mode == 0
            BOUNDARY = [];
        end

        pose = circumnavigate(r, goal, pose);
        if circumnavigate_ok == 0 % We finished circumnavigation, and need to
                                  % go forward
            if work_mode == 0
                BOUNDARY = [];
            end
        elseif circumnavigate_ok == -1
            display('TRAPPED')
            SetFwdVelRadiusRoomba(r, 0, inf); % stop iCreate
            if work_mode == 0
                boundary_done = true;
            end

            end_pose = pose;
            return
        elseif circumnavigate_ok == 1
            % Remember to stop the robot
            display('SUCCEED')
            SetFwdVelRadiusRoomba(r, 0, inf);

            end_pose = pose;
            return
        end
    end
end
