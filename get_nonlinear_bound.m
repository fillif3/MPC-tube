function B_set= get_nonlinear_bound(omega,L,dim)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
A=omega.A;
B=omega.b;
A=[A,zeros(size(A));zeros(size(A)),A];
B=[B;B];
func = @(x) -norm(x(1:dim)-x((dim+1):(2*dim)));
[x,fval] = ga(func,dim*2,A,B);%zeros(12,1)
B_set = Polyhedron('lb',L*fval*ones(1,dim),'ub',-L*fval*ones(1,dim));
end