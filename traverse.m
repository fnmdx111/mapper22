function traverse(r, old_pose)
    display('traversing!!!');

    global is_traversing_subpath
    is_traversing_subpath = false;

    global START
    global END
    global VISITED

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

    targets = [];
    for i = 1:size(START, 1)
        targets(end+1, :) = [START(i, 2) START(i, 1)];
        targets(end+1, :) = [END(i, 2) END(i, 1)];
    end

    trplot2(se(targets(1,1), targets(1,2),0), 'color', 'blue');
    trplot2(se(targets(2,1), targets(2,2), 0), 'color', 'yellow');

    pose = old_pose;
    for i = 1:size(targets, 1)
        trplot2(se(targets(i,1), targets(i,2), 0), 'color', 'blue');
        trplot2(pose, 'color', 'black');

        is_traversing_subpath = true;

        pose = bug2(r, pose, targets(i, 1), targets(i, 2), 1);
    end
end
