% `circumnavigate` will work even without wall sensor
function pose = circumnavigate(r, goal, old_pose)
    trplot2(old_pose, 'color', 'red');
    global circumnavigate_ok

    global boundary_done
    global BOUNDARY

    % mw_g stands for module-wise global variable
    global mw_g_circumnavigate_goal_coord
    mw_g_circumnavigate_goal_coord = goal;

    % gw_g stands for global-wise global variable
    global gw_g_bug2_tolerance % master tolerance in bug2-related functions
    global mw_g_circumnavigate_tolerance
    mw_g_circumnavigate_tolerance = gw_g_bug2_tolerance + 0.05;

    origin = old_pose;
    pose = origin;

    global mw_g_circumnavigate_obstacle_hit_pos
    mw_g_circumnavigate_obstacle_hit_pos = pos_from_ht(pose);

    x1 = mw_g_circumnavigate_obstacle_hit_pos(1);
    y1 = mw_g_circumnavigate_obstacle_hit_pos(2);
    x2 = mw_g_circumnavigate_goal_coord(1);
    y2 = mw_g_circumnavigate_goal_coord(2);

    mk = (y1 - y2) / (x1 - x2);
    mb = (x1 * y2 - x2 * y1) / (x1 - x2);

    global mw_g_circumnavigate_mline_parameters
    mw_g_circumnavigate_mline_parameters = [mk, mb];

    % we need this position every time we want to tell if the
    % robot is at the obstacle hit point again,
    % we also need the distance from obstacle to the goal, which
    % we only calculate once
    if boundary_done == false
        BOUNDARY(end+1, :) = mw_g_circumnavigate_obstacle_hit_pos;
    end

    global mw_g_circumnavigate_obstacle_to_goal_dist
    mw_g_circumnavigate_obstacle_to_goal_dist = norm(mw_g_circumnavigate_obstacle_hit_pos - goal);

    % ensure Create will not stop at first few steps 
    while norm(pose(:, 3) - origin(:, 3)) <= mw_g_circumnavigate_tolerance
        pose = next_move(r, pose); % update position
    end

    % move before Create comes back to the point where it first
    % hit the wall
    f = am_i_done(r, pose);
    while f == 999
        pose = next_move(r, pose); % update position

        f = am_i_done(r, pose);
        if f ~= 999
            break
        end
    end

    circumnavigate_ok = f;
    % if ok == 0, forward we go
    % if ok == 1, goal we are at
    % if ok == -1, trapped we must be
    % else (f == 999), go on circumnavigation
end

function finish = am_i_done(r, pose)
    trap_tolerance = 0.19;
    global mw_g_circumnavigate_tolerance
    global mw_g_circumnavigate_goal_coord
    global mw_g_circumnavigate_obstacle_hit_pos
    global mw_g_circumnavigate_obstacle_to_goal_dist
    global mw_g_circumnavigate_mline_parameters

    current_pos = pos_from_ht(pose);
    dist = norm(mw_g_circumnavigate_goal_coord - current_pos);

    if dist < mw_g_circumnavigate_tolerance
        finish = 1;
        return
    elseif norm(mw_g_circumnavigate_obstacle_hit_pos - current_pos) < trap_tolerance
        finish = -1; % we are back at where we start circumnavigating
        return       % we are most certainly trapped
    elseif is_intersected(mw_g_circumnavigate_mline_parameters, pose) == 1
        if dist < mw_g_circumnavigate_obstacle_to_goal_dist
            finish = 0;
            return;
        end
    end
    finish = 999;
end

% move, and update the position of Create
function pose = next_move(r, old_pose)
    % the central idea is that
    % * we walk along the wall if there is a wall,
    % * we stop at bumps and bypass them by turning left and moving forward
    %     for a short distance,
    % * we turn right and move forward for a short distance if we can't
    %     find a wall.
    global boundary_done
    global BOUNDARY
    global is_traversing_subpath
    global OBS_BOUNDARY

    wall = wall_test(r);
    bump = bump_test(r);

    if bump ~= NO_BUMP   % if Create bumps into a something, it must stop
        if bump == FRONT
            pose = old_pose * turn_left_till_bump_gone(r);
        else
            pose = old_pose * bypass(r, old_pose);
        end
    else
        if wall == 1
            pose = old_pose * walk_straightly(r);

            if boundary_done == false
                BOUNDARY(end + 1, :) = pos_from_ht(pose);
            end

            if is_traversing_subpath == true
                OBS_BOUNDARY(end+1, :) = pos_from_ht(pose);
            end
        else
            pose = old_pose * turn_right_until_a_wall(r, old_pose);
        end
    end
end

function pose = turn_left_till_bump_gone(r) 
    bump = bump_test(r);

    angle_accum = AngleSensorRoomba(r);
    
    if bump ~= NO_BUMP
        DistanceSensorRoomba(r);
    end
    % As a matter of fact, one can only accumulate angles relative to
    % prior movements, instead of relative to the point when it just
    % begins to turn (you'll miss the angles ghostly turned during the
    % last movement).

    % These values that were discarded earlier play an important role in
    % producing errors during navigation.

    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while bump ~= NO_BUMP
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r); 

        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum);
end

function pose = turn_right_until_a_wall(r, old_pose)
    global simulator
    global boundary_done
    global BOUNDARY
    global is_traversing_subpath
    global OBS_BOUNDARY

    angle_accum = AngleSensorRoomba(r);

    wall = wall_test(r);
    bump = bump_test(r);

    max_angle = 33 * pi / 180; % a threshold which controls the
                               % maximum turning angle that ensures
                               % that Create will not end up being in
                               % the wrong direction

                               % This value shouldn't be too large:
                               %   it would lead Create to the wrong
                               %   direction.
                               % Nor should it be too small:
                               %   the distance error incurred in turning
                               %   right will be unacceptible.
    if simulator == 0 % Create tends to turn more than we want it to
        max_angle = 11 * pi / 180;
    end

    delta_pose = se(0., 0., 0.);

    max_angle_reached = 0;

    while and((wall == 0), (bump == NO_BUMP)) % if we bumped into something
                                           % we most certainly shouldn't move
        SetFwdVelRadiusRoomba(r, TURN_VEL, -eps);

        pause(0.2)

        angle = AngleSensorRoomba(r);
        angle_accum = angle_accum + angle;

        if max_angle < abs(angle_accum)
            % If we have turned by this angle,
            % just stop turning, because there
            % is a big chance that we won't find
            % the next wall if we continue to turn,
            % so we instead walk ahead a few steps,
            % and then start turning again.
            dist_accum = DistanceSensorRoomba(r);
            % dist_accum = DistanceSensorRoomba(r); % TODO!!!
   
            while dist_accum < BYPASS_DIST % walk ahead
                % TODO: Maybe there will be a case that we stop here?
                % FIXED: Every time we walked a bit ahead, the distance
                %        will be recalculated again, so no need to test
                %        the distance here.
                dist_delta = move_forward(r, WALK_VEL, WALK_TIME);
                dist_accum = dist_accum + dist_delta;

                if boundary_done == false
                    boundary_new_row = old_pose * se(dist_accum, 0, angle_accum);
                    boundary_new_row = pos_from_ht(boundary_new_row);
                    BOUNDARY(end+1, :) = boundary_new_row;
                end  
                if is_traversing_subpath == true
                    OBS_BOUNDARY_new_row = old_pose * se(dist_accum, 0, angle_accum);
                    OBS_BOUNDARY_new_row = pos_from_ht(OBS_BOUNDARY_new_row);
                    OBS_BOUNDARY(end+1, :) = OBS_BOUNDARY_new_row;
                end

                ok = am_i_done(...
                    r,...
                    old_pose * se(dist_accum, 0, angle_accum));

                if ok ~= 999
                    pose = se(dist_accum, 0, angle_accum + AngleSensorRoomba(r));
                    return
                end

                bump_ = bump_test(r);
                if bump_ ~= NO_BUMP
                    break;
                end
            end

            ok = am_i_done(...
                r,...
                old_pose * se(dist_accum, 0, angle_accum));
            
            if ok ~= 999
                pose = se(dist_accum, 0, angle_accum + AngleSensorRoomba(r));
                return
            end

            delta_pose = delta_pose *...
                         se(dist_accum, 0.,...
                            angle_accum + AngleSensorRoomba(r));

            max_angle_reached = 1;
            break
        end

        wall = wall_test(r);
        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);
    %display('velocity be 0!!!!!!');

    if max_angle_reached == 1
        angle_accum = 0.;
        % if maximum angle was reached, there's no need in
        % accumulating the angle to the pose again
    end

    pose =  delta_pose * se(0.,...
                            0.,...
                            angle_accum + AngleSensorRoomba(r));
end

function new_pose = bypass(r, old_pose)
    global boundary_done
    global BOUNDARY
    global is_traversing_subpath
    global OBS_BOUNDARY

    angle_accum = AngleSensorRoomba(r);

    bump = bump_test(r);

    % turn left until bumps are gone
    SetFwdVelRadiusRoomba(r, TURN_VEL, eps);
    while bump ~= NO_BUMP
        pause(0.2)

        angle_accum = angle_accum + AngleSensorRoomba(r);

        bump = bump_test(r);
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    new_pose = se(0., 0., angle_accum);

    dist_accum = 0.;
    while dist_accum < BYPASS_DIST % walk ahead
        dist_delta = move_forward(r, WALK_VEL, WALK_TIME);
        dist_accum = dist_accum + dist_delta;

        if boundary_done == false
            boundary_new_row = old_pose * se(dist_accum, 0, angle_accum);
            boundary_new_row = pos_from_ht(boundary_new_row);
            BOUNDARY(end+1, :) = boundary_new_row;
        end  
        if is_traversing_subpath == true
            OBS_BOUNDARY_new_row = old_pose * se(dist_accum, 0, angle_accum);
            OBS_BOUNDARY_new_row = pos_from_ht(OBS_BOUNDARY_new_row);
            OBS_BOUNDARY(end+1, :) = OBS_BOUNDARY_new_row;
        end

        ok = am_i_done(...
            r,...
            old_pose * se(dist_accum, 0, angle_accum));
        if ok ~= 999
            new_pose = new_pose * se(dist_accum, 0., AngleSensorRoomba(r));
            return
        end

        bump_ = bump_test(r);
        if bump_ ~= NO_BUMP
            break;
        end
    end

    new_pose = new_pose * se(dist_accum, 0., AngleSensorRoomba(r));
                                             % Accumulate the angle
                                             % every time!
end
