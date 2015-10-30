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

    x_diff = 0.1;
    y_diff = 0.1;

    endpoints = [];

    % plan from Y+ to Y-
    next_grid_coord_y = maxy;
    endpoint_count = 0;
    while next_grid_coord_y >= miny
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
        %                           minx point = a, maxx point = b,
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

        next_grid_coord_y = next_grid_coord_y - diam;
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
                %   3. the y of the minx coord, i.e. current method
                %   4. the average y of all the ys (of the points)
                %      in the grid
                %   5. ...
                minp = b;
            end
            if maxx < x
                maxx = x;
                maxp = b;
            end
        end
    end
end