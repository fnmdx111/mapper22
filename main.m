function main(r)
    global simulator
    simulator = 1;

    global DIAMETER
    DIAMETER = 0.4;

    global tolerance
    tolerance = 0.25;

    global VISITED
    global OBS_BOUNDARY
    OBS_BOUNDARY = [];

    p = find_boundary(r);

    plan();

    traverse(r, p);

    global BOUNDARY
    figure
    env_plot(BOUNDARY, 0, 'green');
    env_plot(VISITED, 0, 'green');
    env_plot(OBS_BOUNDARY, 0, 'green');
end
