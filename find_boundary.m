function pose = find_boundary(r)
    global boundary_done
    global BOUNDARY
    
    boundary_done = false;


    global circumnavigate_ok

    %origin = se(0,0,0);
    endpose = se(2000000,0,0);

    global goal_coord
    goal_coord = pos_from_ht(endpose);

    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));

    while boundary_done == false
        pose=turn_towards_dest(r,pose);
        display(pose)

%         CALIBRATE_COUNTER = 5;
%         counter = 0;

        %move forward until bump
        bump=bump_test(r);
        while bump==NO_BUMP
            display(norm(pos_from_ht(pose) - goal_coord))

            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);

            bump = bump_test(r);
        end

        %circumnavigate
        %if arrive end point--exit
        %if arrive last bump point--exit,failure
        %if meet the intersected line, break and turn towards end point
        BOUNDARY = [];
        pose = circumnavigate(r, pose);
        if circumnavigate_ok == 0 % We finished circumnavigation, and need to
                                  % go forward, so discard previous BOUNDARY
            BOUNDARY = [];
        elseif circumnavigate_ok == -1
            display('Boundary founded')
            display(BOUNDARY)
            SetFwdVelRadiusRoomba(r, 0, inf); % stop iCreate
            boundary_done = true;
        end
    end
end

    
    
    
    
    