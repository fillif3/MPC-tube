function cost=cost_following_trajectory(inputs,state,model,Q,R,nominal_velocity,lambda,...
    horizon,number_of_inputs,nonlinear_constraint_function)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
cost=0;
for i=1:horizon
    input = inputs((1+(i-1)*number_of_inputs):(i*number_of_inputs))';
    
    state = model(state,input);
    input(1) = input(1)-nominal_velocity;
    cost= cost+state'*Q*state+input'*R*input+lambda*max(nonlinear_constraint_function(state));
end
end