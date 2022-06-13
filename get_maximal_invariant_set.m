function set= get_maximal_invariant_set(X_set,A,B,Q,R)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
K=dlqr(A,B,Q,R);
Ak=A-K*B;
set=X_set;
while true
    set=Ak*set;
    if set < X_set
        break
    end
    A_new = [set.A;X_set.A];
    b_new = [set.b;X_set.b];
    set = Polyhedron(A_new,b_new);
end