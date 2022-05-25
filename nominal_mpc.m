function input = nominal_mpc(state,trajectory,obstacles,model,Q,R,S,horizon,...
    lambda,number_of_inputs,max_y_error,a,b,max_alfa,previous_input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nonlinear_constraint_function = @(state) safe_error(state,obstacles,max_y_error,max_alfa,a,b);
cost_function= @(input) cost_following_trajectory(input,state,model,trajectory,...
    Q,R,S,lambda,horizon,number_of_inputs,previous_input,nonlinear_constraint_function);



end