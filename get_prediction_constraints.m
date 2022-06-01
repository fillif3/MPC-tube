function [F,H] = get_prediction_constraints(A,b,horizon)
F=A;
for i=1:(horizon-1)
    F=blkdiag(F,A);
end
H=repmat(b,[horizon,1]);
end