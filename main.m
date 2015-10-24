function main(r)

global DIAMETER
DIAMETER = 0.4;

global simulator
simulator = 1;

find_boundary(r);
arrange();
traverse(r);
env_plot();

end




