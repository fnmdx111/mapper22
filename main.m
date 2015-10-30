function main(r)

    global DIAMETER
    DIAMETER = 0.4;

    global simulator
    simulator = 1;

    global tolerance
    tolerance = 0.25;

    global START
    global END
    global VISITED
    global ENDPOINTS

    global USE_PLAN
    USE_PLAN = 1;

    p = find_boundary(r);

    global BOUNDARY
    env_plot(BOUNDARY, 0, 'red');

    display('find boundary done..........')

    if USE_PLAN == 1
        plan();

        env_plot(ENDPOINTS, 0, 'blue');
    else
        arrange();

        env_plot(START, 0, 'blue');
        hold on
        env_plot(END, 0, 'red');
        hold on
    end

    traverse(r, p);

    env_plot(VISITED, 0, 'green');
end
