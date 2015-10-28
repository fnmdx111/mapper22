function bug2(startpose, endpose, status)

% status: 0 - found boundary
%         1 - traverse

global circumnavigate_ok
global tolerance

global goal_coord
goal_coord = pos_from_ht(endpose);

pose=startpose;

global current_start
current_start = startpose;


global boundary_done
global BOUNDARY

if (status == 0)
    boundary_done = false;
end

global VISITED
    
while true
    pose=turn_towards_dest(r,pose);
    %display(pose)
    
    %move forward until bump
    bump=bump_test(r);
    while bump==NO_BUMP
        %display(norm(pos_from_ht(pose) - goal_coord))
       
        dist = move_forward(r, WALK_VEL, WALK_TIME);
        pose = pose * se(dist, 0, 0);
        
        if (status == 1)
            visited_new_row = pos_from_ht(pose);
            VISITED(end+1,:) = visited_new_row;
        end
        
        if norm(pose(:, 3) - endpose(:,3)) < tolerance
            display('SUCCEED')
            SetFwdVelRadiusRoomba(r, 0, inf);
            circumnavigate_ok = 1;
            return;
        end
        
        bump = bump_test(r);
    end
    
    %circumnavigate
    %if arrive end point--exit
    %if arrive last bump point--exit,failure
    %if meet the intersected line, break and turn towards end point
    if (status == 0)
        BOUNDARY = [];
    end
    pose = circumnavigate(r, pose);
    if circumnavigate_ok == 0 % We finished circumnavigation, and need to
                              % go forward
    if (status == 0)
        BOUNDARY = [];
    end
    elseif circumnavigate_ok == -1
        SetFwdVelRadiusRoomba(r, 0, iwnf); % stop iCreate
        if (status == 0)
            boundary_done = true;
        end
        return
    elseif circumnavigate_ok == 1
        % Remember to stop the robot
        display('SUCCEED');
        SetFwdVelRadiusRoomba(r, 0, inf);
        return
    end
end


end
