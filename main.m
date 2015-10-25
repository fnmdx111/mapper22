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

find_boundary(r);
display('find boundary done..........')
arrange();
display('arrange done..........')
traverse(r);
display('traverse done..........')

%VISITED = BOUNDARY;

env_plot();
display('plot done..........')

end




