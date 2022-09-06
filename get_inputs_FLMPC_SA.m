function [u,input] = get_inputs_FLMPC_SA(fis,Ak_sys,cost_function,X_set_full,horizon,...
    number_of_inputs,A,b,system_equation_nominal,state,nominal_state,previous_solution,lb,ub)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
nonlocon = @(u) nonlocon_helper(u,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs);
cost_function_full = @(u) cost_function(u)+100000*nonlocon(u);
if isempty(previous_solution)
    input = simulannealbnd(cost_function_full,zeros(1,horizon*number_of_inputs),lb*ones(horizon*number_of_inputs),ub*ones(horizon*number_of_inputs));
else
    input = simulannealbnd(cost_function_full,previous_solution,lb*ones(horizon*number_of_inputs),ub*ones(horizon*number_of_inputs));
end
u=input(1);
end

