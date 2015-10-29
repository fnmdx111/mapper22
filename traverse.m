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

    new_END = [];
    for i = 1:size(START, 1)
        new_END(end+1, :) = START(i, :);
        new_END(end+1, :) = END(i, :);
    end

    trplot2(se(new_END(1,1), new_END(1,2),0), 'color', 'blue');
    trplot2(se(new_END(2,1), new_END(2,2), 0), 'color', 'yellow');

    current_start = pos_from_ht(old_pose);

    pose = old_pose;
    for i = 1:size(new_END, 1)
        trplot2(se(new_END(i,1), new_END(i,2), 0), 'color', 'blue');
        trplot2(pose, 'color', 'black');

        is_traversing_subpath = true;

        pose = bug2(r, pose, new_END(i, 1), new_END(i, 2), 1);
    end
end
