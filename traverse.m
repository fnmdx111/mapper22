function traverse(r, pose)

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


global circumnavigate_ok

global tolerance;

global goal_coord

new_END = [];
for i = 1:size(START, 1)
    new_END(end+1, :) = START(i, :);
    new_END(end+1, :) = END(i, :);
end

trplot2(se(new_END(1,1), new_END(1,2),0), 'color', 'blue');
trplot2(se(new_END(2,1), new_END(2,2), 0), 'color', 'yellow');

global current_start
current_start = pose;

for i = 1:size(new_END, 1)
    i
    trplot2(se(new_END(i,1), new_END(i,2),0), 'color', 'blue');
    
    is_traversing_subpath = true;

    goal_coord = new_END(i);
    
    trplot2(current_start, 'color', 'black');

    while is_traversing_subpath == true
        pose=turn_towards_dest(r,pose);
        display(pose)

        %move forward until bump
        bump=bump_test(r);
        while bump==NO_BUMP
            display(norm(pos_from_ht(pose) - goal_coord))
            
            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);

            if is_traversing_subpath == true

                visited_new_row = pos_from_ht(pose);
                VISITED(end+1,:) = visited_new_row;
            end
            

            if norm(pose(1:2, 3) - goal_coord) < tolerance
                display('SUCCEED')
                SetFwdVelRadiusRoomba(r, 0, inf);
                circumnavigate_ok = 1;
                is_traversing_subpath = false;
                break;
            end
            bump = bump_test(r);
        end
        
        if circumnavigate_ok == 1
            is_traversing_subpath = false;
            display('SUCCEED2');
            break;
        end
        
        %circumnavigate
        %if arrive end point--exit
        %if arrive last bump point--exit,failure
        %if meet the intersected line, break and turn towards end point
        
        pose = circumnavigate(r, pose);
        if circumnavigate_ok == 0 % We finished circumnavigation, and need to
            % go forward, so do nothing here
        elseif circumnavigate_ok == 1
            % Remember to stop the robot
            display('SUCCEED3');
            SetFwdVelRadiusRoomba(r, 0, inf);
            is_traversing_subpath = false;
            current_start = pose;

            %break;
        end
    end
    
end


end
