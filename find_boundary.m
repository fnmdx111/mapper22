function pose = find_boundary(r)
    global boundary_done
    global BOUNDARY
    
    boundary_done = false;

    global circumnavigate_ok

    %origin = se(0,0,0);
    end_ = [2000000.0, 0.];

    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));

    pose = bug2(r, pose, end_(1), end_(2), 0);
end
