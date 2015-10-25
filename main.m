function main(r)

global DIAMETER
DIAMETER = 0.4;

global simulator
simulator = 1;

global tolerance
tolerance = 0.25;

<<<<<<< HEAD
=======
global BOUNDARY
global SORT
global START
global END
global VISITED

>>>>>>> 4b1fcf32ad047499ace836130986537278fe3245
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




