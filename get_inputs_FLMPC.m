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
    error('error')
end
u=inputs(1);
end

function [c,ceq]=nonlocon_helper(inputs,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs)
ceq=0;
c=horizon-0.1;
fisout_unlikely = fis{1};
fisout_main = fis{2};

X_set_unlikely = X_set_full{1};
X_set_main = X_set_full{2};

A_set = [eye(length(state));-eye(length(state))];
set = Polyhedron(state'-nominal_state');

for i=1:horizon
    u=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i);
    nominal_state = system_equation_nominal(nominal_state,u);
    set = Ak_sys*set;
    % Check unlikely case
    out = evalfis(fisout_unlikely,abs(nominal_state));
    b_set = [0.1*out;out;0.1*out;out];
    unlikely_set = Polyhedron(A_set,b_set)+set;
    if ~((unlikely_set+unlikely_set)<X_set_unlikely)
        break
    end

    % Check normal case
    out = evalfis(fisout_main,abs(nominal_state));
    b_set = [0.1*out;out;0.1*out;out];
    set = (Polyhedron(A_set,b_set)+Ak_sys*set);
    
    if ~((nominal_state+set)<X_set_main)
        break
    end
    c=c-1;
end
end