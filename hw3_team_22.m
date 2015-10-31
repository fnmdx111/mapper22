%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 3
%
% Team number: 22
% Team leader: Zihang Chen (zc2324)
% Team members: Yixing Chen (yc3094), Xin Yang (xy2290)
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% This file is concatenated automatically by <concat.py>.

% This function is generated automatically by <concat.py>.
function hw3_team_22(r)
    global simulator
    simulator = 1;

    main(r);
end

% <concat.py>: concatenating main.m ------->
function main(r)
    global simulator

    global DIAMETER
    DIAMETER = 0.4;

    global tolerance
    tolerance = 0.25;

    global VISITED
    global OBS_BOUNDARY
    OBS_BOUNDARY = [];

    p = find_boundary(r);

    plan();
    global ENDPOINTS
    env_plot(ENDPOINTS, 0, 'blue');

    traverse(r, p);
    display('Traverse done, plotting, please wait...')

    global BOUNDARY
    figure
    env_plot(BOUNDARY, 0, 'green');
    env_plot(VISITED, 0, 'green');
    % env_plot(OBS_BOUNDARY, 0, 'green');
end

% <concat.py>: concatenating bug2.m ------->
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

% <concat.py>: concatenating bump_test.m ------->
function bump = bump_test(r)
    [br, bl, ~, ~, ~, bf] = BumpsWheelDropsSensorsRoomba(r);

    if bl == 1
        bump = LEFT;
    elseif br == 1
        bump = RIGHT;
    elseif bf == 1
        bump = FRONT;
    else
        bump = NO_BUMP;
    end
end

% <concat.py>: concatenating BYPASS_DIST.m ------->
function h = BYPASS_DIST
% BYPASS_DIST controls how long by walking forward
% the robot bypasses current obstacle

% If this value is too much, we are too far away from
% the obstacle, and stand little chance of getting
% back to the obstacle (because we're circumnavigating).

% And if it's too little, the robot won't move at all
% due to frictions.

h = 0.005;

end


% <concat.py>: concatenating circumnavigate.m ------->
% `circumnavigate` will work even without wall sensor
function pose = circumnavigate(r, goal, old_pose)
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

% <concat.py>: concatenating dist_to_dest.m ------->
function dist = dist_to_dest(old_pose)
    global goal_coord % [4 0] destination

    current_pos = pos_from_ht(old_pose);

    dist = norm(goal_coord - current_pos);
end


% <concat.py>: concatenating env_plot.m ------->
function env_plot(points, fig, color)
  global DIAMETER
  diam = DIAMETER;

  vst = points;

  min_y = min(vst(:, 2));
  min_x = min(vst(:, 1));

  nv = [];
  for i = vst'
    nv(end + 1, :) = i - [min_x min_y]' + [diam / 2 diam / 2]';
    display(nv(end, :))
  end % translate all the points in vst to the first quadrant

%   figure
%   set(gca, 'xtick', [0:diam:(max(nv(:, 1)) + diam)]);
%   set(gca, 'ytick', [0:diam:(max(nv(:, 2)) + diam)]);
%   grid on

  for i = nv'
    x = floor(i(1) / diam);
    y = floor(i(2) / diam);

%     if abs(i(1) - x * diam) < 0.1
%         rectangle('Position', [(x - 1)*diam y*diam diam diam],...
%                   'FaceColor', color);
%     end
    if abs(i(2) - y * diam) < 0.1
        rectangle('Position', [x*diam (y - 1)*diam diam diam],...
                  'FaceColor', color);
    end
    if abs(i(2) - (y + 1) * diam) < 0.1
        rectangle('Position', [x*diam (y + 1)*diam diam diam],...
                  'FaceColor', color);
    end

%    set(0, 'CurrentFigure', fig);
    rectangle('Position', [x*diam y*diam diam diam],...
              'FaceColor', color);
  end
end

% <concat.py>: concatenating find_boundary.m ------->
function pose = find_boundary(r)
    global boundary_done
    boundary_done = false;

    %origin = se(0,0,0);
    end_ = [2000000.0, 0.];

    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));

    pose = bug2(r, pose, end_(1), end_(2), 0);
end

% <concat.py>: concatenating FRONT.m ------->

function v = FRONT
    v = 1;
end

% <concat.py>: concatenating is_intersected.m ------->
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

% <concat.py>: concatenating LEFT.m ------->

function v = LEFT
    v = 2;
end

% <concat.py>: concatenating move_forward.m ------->
function dist = move_forward(r, vel, time)
    init = DistanceSensorRoomba(r);
    SetFwdVelRadiusRoomba(r, vel, inf);
    pause(time)

    dist = DistanceSensorRoomba(r) + init;
end

% <concat.py>: concatenating NO_BUMP.m ------->

function v = NO_BUMP
  v = 0;
end

% <concat.py>: concatenating plan.m ------->
function plan
    global BOUNDARY
    global DIAMETER
    global ENDPOINTS

    global mw_g_plan_grid_cell_tol
    mw_g_plan_grid_cell_tol = 0.1;

    global mw_g_plan_diam
    mw_g_plan_diam = DIAMETER;

    diam = mw_g_plan_diam;

    % plan trajectory about axis Y
    maxy = max(BOUNDARY(:, 2));
    miny = min(BOUNDARY(:, 2));

    x_diff = 0.15;
    y_diff = 0.15;

    endpoints = [];

    % plan from Y+ to Y-
    next_grid_coord_y = maxy - diam * 0.4;
    endpoint_count = 0;
    while next_grid_coord_y > miny
        % find the maximum x and minimum x on the grid lines defined by
        % y = next_grid_coord_y, i.e.
        %
        % In this graph, each vertex or outer edge (denoted by + or -)
        % may represent a point on boundary, the
        % size of each grid is of one diam x one diam.
        %
        % **+-+   +---+---+--b+      x
        % */  |   |   |   |   |      ^
        % +---+-c-+---+---+---+---+  |
        % |   |   |   |   |   |   |  +---> y
        % +---+---+---+---+---+---+*+-+
        %     |   |   |   |   |   |/ /*
        %     +d--+   +---+a--+---+-+-*
        %         ^           ^
        %         |           |
        %         |           |
        %         |           +-- Say y = 3.141,
        %         |               we find in this grid line:
        %         |                 minx point = a, maxx point = b,
        %         |               so we add [ax ay] [bx by] to endpoints
        %         +-- Say y = 1.571
        %             we find in this grid line:
        %               minx point = d, maxx point = c,
        %             so we add [dx dy] [cx cy] to endpoints
        %
        % Now we have [a b ... d c ...] in our endpoints, to output the
        % final ENDPOINTS, we have to reorder endpoints like this:
        %   For each a group of four elements in endpoints:
        %   1. get the first element of endpoints, push;
        %   2.   if there are more elements,
        %      get the second element of endpoints, push;
        %   3. if there are more elements,
        %      get the fourth element of endpoints, push;
        %   4. if there are more elements,
        %      get the third element of endpoints, push;
        %
        % E.g. endpoints [a b d c] => [a b c d]

        [minp, maxp] = find_about_axis_x(next_grid_coord_y);

        endpoints(end + 1, :) = minp;
        endpoint_count = endpoint_count + 1;
        if maxp(1) - minp(1) >= mw_g_plan_grid_cell_tol
            % if maxx - minx <= epsilon, they are considered as
            % the same cell
            endpoints(end + 1, :) = maxp;
            endpoint_count = endpoint_count + 1;
        end

        next_grid_coord_y = next_grid_coord_y - diam * 0.8;
    end

    i = 1;
    ENDPOINTS = zeros(endpoint_count, 2);
    while true
        if i <= endpoint_count
            ENDPOINTS(i, :) = endpoints(i, :);
            ENDPOINTS(i, 1) = ENDPOINTS(i, 1) + x_diff;
        end

        if i + 1 <= endpoint_count
            ENDPOINTS(i + 1, :) = endpoints(i + 1, :);
            ENDPOINTS(i + 1, 1) = ENDPOINTS(i + 1, 1) - x_diff;
        end

        if i + 3 <= endpoint_count
            ENDPOINTS(i + 2, :) = endpoints(i + 3, :);
            ENDPOINTS(i + 2, 1) = ENDPOINTS(i + 2, 1) - x_diff;
        end

        if i + 2 <= endpoint_count
            ENDPOINTS(i + 3, :) = endpoints(i + 2, :);
            ENDPOINTS(i + 3, 1) = ENDPOINTS(i + 3, 1) + x_diff;
        end

        i = i + 4;
        if i > endpoint_count;
            break;
        end
    end
    ENDPOINTS(1, 2) = ENDPOINTS(1, 2) - y_diff;
    if size(ENDPOINTS, 1) > 1
        ENDPOINTS(2, 2) = ENDPOINTS(2, 2) - y_diff;
    end
    if size(ENDPOINTS, 1) == 3
        ENDPOINTS(end, 2) = ENDPOINTS(2, 2) + y_diff;
    elseif size(ENDPOINTS, 1) > 3
        ENDPOINTS(end - 1, 2) = ENDPOINTS(end - 1, 2) + y_diff;
        ENDPOINTS(end, 2) = ENDPOINTS(end, 2) + y_diff;
    end
end

function [minp, maxp] = find_about_axis_x(coord_y)
    global BOUNDARY
    global mw_g_plan_grid_cell_tol
    global mw_g_plan_diameter

    minx = 9999999.;
    minp = [0 0];
    maxx = -9999999.;
    maxp = [0 0];
    for b = BOUNDARY'
        x = b(1); y = b(2);
        if and(y >= coord_y - mw_g_plan_grid_cell_tol,...
               y < coord_y)
            if minx > x
                minx = x;
                % For the x coord, we are fixed with minx and maxx,
                % but for the y coord, we have multiple choices:
                %   1. the middle y (this point may not be in the BOUNDARY
                %      point set) of the grid
                %   2. the rightmost y of the grid, i.e. coord_y
                %   3. the y of the minx coord
                %   4. the average y of all the ys (of the points)
                %      in the grid
                %   5. ...
                minp = [minx, y];
            end
            if maxx < x
                maxx = x;
                maxp = [maxx, y];
            end
        end
    end
end

% <concat.py>: concatenating pos_from_ht.m ------->
function v = pos_from_ht(ht)
%POS_FROM_HT Summary of this function goes here
%   Detailed explanation goes here
    v = [ht(1, 3) ht(2, 3)];

end


% <concat.py>: concatenating RIGHT.m ------->
function v = RIGHT
  v = 3;
end

% <concat.py>: concatenating se.m ------->
% homogeneous coordinating
function pose = se(x, y, theta)
    pose = [cos(theta), -sin(theta), x;
            sin(theta), cos(theta), y;
            0, 0, 1];
end

% <concat.py>: concatenating traverse.m ------->
function traverse(r, old_pose)
    display('traversing!!!');

    global is_traversing_subpath
    is_traversing_subpath = false;

    global START
    global END
    global VISITED
    global ENDPOINTS

    START_size = size(START);
    END_size = size(END);

    if or(START_size(1) ~= END_size(1), START_size(2) ~= END_size(2))
        display('Error. Size of START and END different.');
        return
    end
    if START_size(2) ~= 2
        display('Error. Size of START and END is not n * 2');
    end

    VISITED = [];

    targets = ENDPOINTS;


    pose = old_pose;
    for i = 1:size(targets, 1)
        display(pos_from_ht(pose));
        display(targets(i));

        is_traversing_subpath = true;

        pose = bug2(r, pose, targets(i, 1), targets(i, 2), 1);
    end
end

% <concat.py>: concatenating turn_towards_dest.m ------->
function pose = turn_towards_dest(r, goal_coord, old_pose)
% turn the iCreate to make it head towards destination

% The basic idea here is that we calculate the dot product between
% the unit vector from start (origin) to goal, and the unit vector
% of current orientation.
% If the dot product result is nearly one, we are aligned to the
% goal;
% otherwise, turn a little and calculate the dot product again

% A little optimization here is that using the one of the normal
% vectors of the destination vector (from start to goal), we can
% minimize the angle we turn. If dot product between the orientation
% vector and the selected normal vector is bigger than 0, then we
% know that by turning right, the angle we turn will be less than
% 180 degrees and vice versa.
% The trick here is to test out the corresponding normal vector and
% the orientation we turn to.

rob_vec = [old_pose(1, 1) old_pose(2, 1)]';

DOT_PROD_TOLERANCE = 0.0005;

FULL_ALIGN = 1;
CALIBRATE_TOL = FULL_ALIGN - DOT_PROD_TOLERANCE;

dest_vec = goal_coord' - pos_from_ht(old_pose)';
dest_vec = dest_vec / norm(dest_vec);

dest_norm_vec = norm_vec(dest_vec);

display('yiiiiiiipeeeeeeeee')
display(dot(rob_vec, dest_vec))
display(dot(dest_norm_vec, rob_vec));

if dot(dest_norm_vec, rob_vec) > 0 % we need to make iCreate turn left

    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL / 3, eps);
    while dot(rob_vec, dest_vec) < CALIBRATE_TOL % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);
        display(dot(rob_vec, dest_vec))

        angle_accum = angle_accum + angle;
    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
    pose = old_pose * pose;

else % current_angle > 0, we need to make iCreate turn right

    angle_accum = AngleSensorRoomba(r);

    SetFwdVelRadiusRoomba(r, TURN_VEL / 3, -eps);
    while dot(rob_vec, dest_vec) < CALIBRATE_TOL % TODO: need to consider bump???
        pause(0.1)

        angle = AngleSensorRoomba(r);
        rob_vec = rot(rob_vec, angle);
        display(dot(rob_vec, dest_vec))

        angle_accum = angle_accum + angle;

    end
    SetFwdVelRadiusRoomba(r, 0, inf);

    pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
    pose = old_pose * pose;
end

end

function new_v = rot(v, rad)
    new_v = [cos(rad) -sin(rad); sin(rad) cos(rad)] * v;
end

function v = norm_vec(dv)
    v = [dv(2), -dv(1)]';
end

% <concat.py>: concatenating TURN_VEL.m ------->
function h = TURN_VEL
%TURN_VEL Summary of this function goes here
%   Detailed explanation goes here

    global simulator

    if simulator
        h = 0.001; % if we set h = 0.1 on the simulator, it just blows up
    else
        h = 0.05;
    end
end


% <concat.py>: concatenating walk_straightly.m ------->
function pose = walk_straightly(r)
    dist_accum = 0.;

    pose = se(0, 0, AngleSensorRoomba(r));
    % better include this angle

    dist_accum = dist_accum + move_forward(r, WALK_VEL, WALK_TIME);

    pose = pose * se(dist_accum, 0, AngleSensorRoomba(r));
end

% <concat.py>: concatenating WALK_TIME.m ------->
function h = WALK_TIME
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


  h = 0.05;

end


% <concat.py>: concatenating WALK_VEL.m ------->
function h = WALK_VEL
%WALK_VEL Summary of this function goes here
%   Detailed explanation goes here

h = 0.4;
end

% <concat.py>: concatenating wall_test.m ------->
function wall = wall_test(r)
    wall = WallSensorReadRoomba(r);
end

