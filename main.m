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
    display('find boundary done..........')

    if USE_PLAN == 1
        plan();

        VISITED = ENDPOINTS;
        env_plot('blue');
    else
        arrange();

        VISITED = START;
        env_plot('blue');
        hold on
        VISITED = END;
        env_plot('red');
        hold on
    end

    traverse(r, p);

    env_plot('green');
end
