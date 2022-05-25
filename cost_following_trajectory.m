function cost=cost_following_trajectory(inputs,state,model,trajectory,Q,R,S,lambda,...
    horizon,number_of_inputs,previous_input,nonlinear_constraint_function)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
cost=0;
for i=1:horizon
    input = inputs((1+(i-1)*number_of_inputs):(i*number_of_inputs))';
    state = model(input,state);
    input_diff = input-previous_input;
    state_diff = state-trajectory{i};
    cost= cost+state_diff'*Q*state_diff+input'*R*input+input_diff'*S*input_diff...
        +lambda*max(nonlinear_constraint_function(state));
    previous_input=input;
end
end