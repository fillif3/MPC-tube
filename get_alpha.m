function alpha = get_alpha(A,W,s)
%   Detailed explanation goes here

number_of_constraints=length(W.b);
alpha=0;
As = A^s;
for i=1:number_of_constraints
    f = W.A(i,:);
    alpha_i = hw(As'*f',W)/W.b(i);
    alpha=max(alpha,alpha_i);
end
end