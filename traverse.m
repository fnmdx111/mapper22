function traverse(r)

global is_traversing
is_traversing = false;
global START
global END
global VISITED
global obstacle_hit_pos

START_size = size(START);
END_size = size(END);

START
END

if (START_size(1) ~= END_size(1)) || (START_size(2) ~= END_size(2))
    display('Error. Size of START and END different.');
    return
end
if START_size(2) ~= 2
    display('Error. Size of START and END is not n * 2');
end

VISITED = [];


global circumnavigate_ok

global tolerance;
tolerance = 0.25;

global goal_coord

new_START = zeros(START_size(1) * 2, 2);
new_END = new_START;

new_START(1,:) = obstacle_hit_pos;
new_END(1,:) = START(1,:);

temp = 1;

for i = 1 : START_size(1)-1
    temp = temp + 1;
    new_START(temp,:) = START(i,:);
    new_END(temp,:) = END(i,:);
    temp = temp + 1;
    new_START(temp,:) = END(i,:);
    new_END(temp,:) = START(i+1,:);
end
temp = temp + 1;
new_START(temp,:) = START(START_size(1),:);
new_END(temp,:) = END(START_size(1),:);


for i = 1 : START_size(1) * 2
    if (new_START(i, 1) == new_END(i, 1)) && (new_START(i, 2) == new_END(i, 2))
        continue;
    end
    
    if i > 1
        is_traversing = false;
    end
    
    
    endpose = se(new_END(i, 1),new_END(i, 2),0);
    goal_coord = pos_from_ht(endpose);
    
    pose=se(DistanceSensorRoomba(r), 0, AngleSensorRoomba(r));
    
    while false
        pose=turn_towards_dest(r,pose);
        display(pose)
        
%         CALIBRATE_COUNTER = 5;
%         counter = 0;
        
        %move forward until bump
        bump=bump_test(r);
        while bump==NO_BUMP
%             if counter > CALIBRATE_COUNTER
%                 % Because it cannot be guaranteed that we exit circumnavigation
%                 % mode with perfect orientation towards goal. And a small error
%                 % in orientation most often turns out to be disastrous. So the
%                 % orientation must be calibrated before it's too late.
%                 % We calibrate our orientation every 2 steps.
%                 display('calibrating-----------------------')
%                 pose = calibrate(r, pose);
%                 if is_traversing == false
%                     visited_new_row = pos_from_ht(pose);
%                     VISTED(end+1,:) = visited_new_row;
%                 end
%                 counter = 0;
%             end
%             counter = counter + 1;
            
            display(norm(pos_from_ht(pose) - goal_coord))
            
            dist = move_forward(r, WALK_VEL, WALK_TIME);
            pose = pose * se(dist, 0, 0);
            if is_traversing == false
                visited_new_row = pos_from_ht(pose);
                VISTED(end+1,:) = visited_new_row;
            end
            
            %trplot2(pose);
            %hold on
            
            if norm(pose(:, 3) - endpose(:,3)) < tolerance
                display('SUCCEED')
                SetFwdVelRadiusRoomba(r, 0, inf);
                circumnavigate_ok = 1;
                break;
            end
            bump = bump_test(r);
        end
        
        if circumnavigate_ok == 1
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
            SetFwdVelRadiusRoomba(r, 0, inf);
            break;
        end
    end
    
end

is_traversing = false;

end
