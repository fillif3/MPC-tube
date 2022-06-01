function set = get_noise_set_from_bounds(bounds)

dim = length(bounds);
A=[eye(dim);-eye(dim)];
b= [bounds;bounds];
set=Polyhedron(A,b);