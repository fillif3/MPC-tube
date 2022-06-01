function [Q_full,R_full]=get_full_weight_matrices(Q_matrix,R_matrix,horizon)
Q_full=Q_matrix;
for i=1:(horizon-1)
    Q_full=blkdiag(Q_full,Q_matrix);
end
R_full=R_matrix;
for i=1:(horizon-1)
    R_full=blkdiag(R_full,R_matrix);
end
end