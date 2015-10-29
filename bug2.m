function end_pose = bug2(r, start_pose, goalx, goaly, work_mode)

    % work_mode: 0 - find boundary
    %            1 - traverse

    global circumnavigate_ok
    global tolerance

    goal = [goalx goaly];

    pose = start_pose;

    global boundary_done
    global BOUNDARY

    if work_mode == 0
        boundary_done = false;
    end

    global VISITED

    while true
        pose = turn_towards_dest(r, goal, pose);

        %move forward until bump
        bump = bump_test(r);
        while bump == NO_BUMP
            %display(norm(pos_from_ht(pose) - goal_coord))

            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);

            if work_mode == 1
                VISITED(end+1, :) = pos_from_ht(pose);
            end

            if norm(pose(1:2, 3) - goal') < tolerance
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
