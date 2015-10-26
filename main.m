function main(r)

global DIAMETER
DIAMETER = 0.4;

global simulator
simulator = 1;

global tolerance
tolerance = 0.25;

global BOUNDARY
global SORT
global START
global END
global VISITED

p = find_boundary(r);
display('find boundary done..........')
arrange();


% figure
% 
VISITED = START;
env_plot('blue');
hold on
VISITED = END;
env_plot('red');
hold on


% display('arrange done..........')
traverse(r, p);
% display('traverse done..........')
% %VISITED = BOUNDARY;

figure

env_plot('blue');
hold on

display('plot done..........')

end




