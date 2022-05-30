function u = nominal_mpc(state,model,Q,R,velocity,horizon,...
    lambda,nonlinear_constraint,max_input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
number_of_inputs=length(max_input);

cost_function= @(input) cost_following_trajectory(input,state,model,...
    Q,R,velocity,lambda,horizon,number_of_inputs,nonlinear_constraint);

ub = repmat(max_input,[number_of_inputs*horizon,1]);%ones(horizon*number_of_inputs)*max_input;
options = optimoptions('ga','Display','off');%,'MaxGenerations',25);


%[inputs,fval,flag,message] = fmincon(cost_function,zeros(1,horizon*number_of_inputs),[],[],[],[],-ub,ub,[],options);
[inputs,fval,flag,message] = ga(cost_function,horizon*number_of_inputs,[],[],[],[],-ub,ub,[],options);

u=inputs(1:number_of_inputs)';

end