function u = get_inputs_FLMPC(fis,Ak_sys,cost_function,X_set_full,horizon,...
    number_of_inputs,A,b,system_equation_nominal,state)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
nonlocon = @(u) nonlocon_helper(u,fis,state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs);
%options = optimoptions('ga','Display','off');
options = optimoptions('ga','MaxGenerations',30,'PlotFcn', @gaplotbestf,'PopulationSize',10);
inputs = ga(cost_function,horizon*number_of_inputs,A,b,[],[],[],[],nonlocon,options);
u=inputs(1);
end

function [c,ceq]=nonlocon_helper(inputs,fis,state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs)
ceq=0;
c=horizon-0.1;

A_set = [eye(length(state));-eye(length(state))];
set = Polyhedron([],[]);
for i=1:horizon
    u=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i);
    out = evalfis(fis,state(1));
    b_set = [0.1*out;out;0.1*out;out];
    if i==1
        set = Polyhedron(A_set,b_set);
    else
        set = Polyhedron(A_set,b_set)+Ak_sys*set;
    end
    state = system_equation_nominal(state,u);
    if ~((state+set)<X_set_full)
        break
    end
    c=c-1;
end
end