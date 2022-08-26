function state = biased_system(A_sys,B_sys,x,u,highest_variance_position,max_var)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
dist= abs(x(1)-highest_variance_position);
%current_var=max_var/(1+dist/5);
current_var=max_var/(1+dist/5)*abs(x(2))/1.5;


state= A_sys*x+B_sys*u+normrnd([0;0],current_var);

end