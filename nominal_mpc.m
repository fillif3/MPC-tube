function u = nominal_mpc(state,trajectory,obstacles,model,Q,R,S,horizon,...
    lambda,number_of_inputs,max_y_error,a,b,max_alfa,previous_input,max_input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nonlinear_constraint_function = @(state) safe_error(state,obstacles,max_y_error,max_alfa,a,b);
cost_function= @(input) cost_following_trajectory(input,state,model,trajectory,...
    Q,R,S,lambda,horizon,number_of_inputs,previous_input,nonlinear_constraint_function);

ub = ones(horizon*number_of_inputs)*max_input;
options = optimoptions('fmincon','Display','off','Algorithm','sqp');


%[inputs,fval,flag,message] = fmincon(cost_function,zeros(1,horizon*number_of_inputs),[],[],[],[],-ub,ub,[],options);
[inputs,fval,flag,message] = ga(cost_function,horizon*number_of_inputs,[],[],[],[],-ub,ub,[],options);

u=inputs(1:number_of_inputs)';

end