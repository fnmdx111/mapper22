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

    p = find_boundary(r);
    display('find boundary done..........')

    arrange();

    VISITED = START;
    env_plot('blue');
    hold on
    VISITED = END;
    env_plot('red');
    hold on

    traverse(r, p);

    env_plot('green');
end
