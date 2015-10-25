function main(r)

global DIAMETER
DIAMETER = 0.4;

global simulator
simulator = 1;

find_boundary(r);
display('find boundary done..........')
arrange();
display('arrange done..........')
traverse(r);
display('traverse done..........')
env_plot();
display('plot done..........')

end




