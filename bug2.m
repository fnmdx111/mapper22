function end_pose = bug2(r, start_pose, goalx, goaly, work_mode)

    % work_mode: 0 - find boundary
    %            1 - traverse

    global circumnavigate_ok

    global gw_g_bug2_tolerance
    gw_g_bug2_tolerance = 0.10;

    goal = [goalx goaly];

    pose = start_pose;

    global boundary_done
    global BOUNDARY

    if work_mode == 0
        boundary_done = false;
    end

    global VISITED

    CALIBRATE_TOLERANCE = 0.04;

    while true
        pose = turn_towards_dest(r, goal, pose);

        OS = pos_from_ht(pose);
        SP = (goal - OS)';
        orientation = [pose(1, 1) pose(2, 1)]';
        sin_theta = sqrt(1 - dot(orientation, SP / norm(SP)) ^ 2);

        % We do this every 3 steps:
        %
        %              * (R_x, R_y)
        %             /|---
        %            / | ^
        %           /  | |
        %          /   | y?
        %   rv -> ^    | |
        %        /)?   | v
        %       *------+---------* (P_x, P_y)
        %       (S_x, S_y)
        %
        %  rv = [pose(1, 1) pose(2, 1)]'
        %  sin? = sqrt(1 - (rv . (SP / ||SP||))^2)
        %  y? = ||RS||sin?
        %  If y? > epsilon, calibrate the orientation!

        step_count = 0;

        bump = bump_test(r);
        while bump == NO_BUMP
            %display(norm(pos_from_ht(pose) - goal_coord))

            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);
            display(norm(pose(1:2, 3) - goal'))

            if step_count > 2
                RS_length = norm(OS - pos_from_ht(pose));
                y = RS_length * sin_theta;
                display(y)

                if y > CALIBRATE_TOLERANCE
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
    
        %circumnavigate
        %if arrive end point--exit
        %if arrive last bump point--exit,failure
        %if meet the intersected line, break and turn towards end point
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
