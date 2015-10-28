function traverse_bug2(r, pose)

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


global current_start

for i = 1:size(new_END, 1)
    i
    
    is_traversing_subpath = true;

    goal_coord = new_END(i);
    current_start = pose;
    
