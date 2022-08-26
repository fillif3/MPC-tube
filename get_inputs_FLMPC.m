function u = get_inputs_FLMPC(fis,Ak_sys,cost_function,X_set_full,horizon,...
    number_of_inputs,A,b,system_equation_nominal,state,nominal_state)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
nonlocon = @(u) nonlocon_helper(u,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs);
%options = optimoptions('ga','Display','off');
options = optimoptions('ga','MaxGenerations',15,'PopulationSize',120);
[inputs,FVAL,EXITFLAG,OUTPUT] = ga(cost_function,horizon*number_of_inputs,A,b,[],[],[],[],nonlocon,options);
disp(inputs)
if EXITFLAG~=1
    kewq=1
end
u=inputs(1);
end

function [c,ceq]=nonlocon_helper(inputs,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs)
ceq=0;
c=horizon-0.1;

A_set = [eye(length(state));-eye(length(state))];
try
    set = Ak_sys*Polyhedron(state'-nominal_state');
catch
    disp(state)
    disp(nominal_state)
end
for i=1:horizon
    u=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i);
    out = evalfis(fis,abs(nominal_state));
    b_set = [0.1*out;out;0.1*out;out];
    set = Ak_sys*(Polyhedron(A_set,b_set)+set);
    nominal_state = system_equation_nominal(nominal_state,u);
    if ~((nominal_state+set)<X_set_full)
        break
    end
    c=c-1;
end
end