function [u,input] = get_inputs_FLMPC_PSO(fis,Ak_sys,cost_function,X_set_full,horizon,...
    number_of_inputs,A,b,system_equation_nominal,state,nominal_state,previous_solution,lb,ub)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
population_size=50;

nonlocon = @(u) nonlocon_helper(u,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs);
cost_function_full = @(u) cost_function(u)+100000*nonlocon(u);
if isempty(previous_solution)
    options = optimoptions('particleswarm','SwarmSize',population_size,'MaxIterations',60);%'MaxStallIterations',10
    input = particleswarm(cost_function_full,horizon*number_of_inputs,lb*ones(horizon*number_of_inputs),ub*ones(horizon*number_of_inputs),options);
else 
    population=create_pso_population(previous_solution,lb,ub,population_size);
    options = optimoptions('particleswarm','SwarmSize',population_size,'MaxIterations',20,'InitialSwarmMatrix',population');%'MaxStallIterations',10
    input = particleswarm(cost_function_full,horizon*number_of_inputs,lb*ones(horizon*number_of_inputs),ub*ones(horizon*number_of_inputs),options);
end    

u=input(1);
end



