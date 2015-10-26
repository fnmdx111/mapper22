function pose = calibrate(r, old_pose)

rob_vec = [old_pose(1, 1) old_pose(1, 2)];

display(rob_vec)

display('dsfsfdsfasdfdsfdsfdsfdfddfdfdfdfdsfsfds')

pos = pos_from_ht(old_pose);
display(pos)

tolerance = 0.075;



if abs(pos(2)) > tolerance
    if pos(2) < 0 % we need to make iCreate turn left
        angle_accum = AngleSensorRoomba(r);

        SetFwdVelRadiusRoomba(r, TURN_VEL / 3, eps);
        pause(0.3) % TODO!!!
        SetFwdVelRadiusRoomba(r, 0, inf);
        
        pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
        pose = old_pose * pose;
    else 
        angle_accum = AngleSensorRoomba(r);

        SetFwdVelRadiusRoomba(r, TURN_VEL / 3, -eps);
        pause(0.3) % TODO!!!
        SetFwdVelRadiusRoomba(r, 0, inf);
        
        pose = se(0, 0, angle_accum + AngleSensorRoomba(r));
        pose = old_pose * pose;
    end
             
end

pose = old_pose;


end

function new_v = rot(v, rad)
    new_v = v * [cos(rad) -sin(rad); sin(rad) cos(rad)];
end

function v = norm_vec(dv)
    v = [dv(2), -dv(1)];
end


    