function [c,ceq]=nonlocon_helper(inputs,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs)
ceq=0;
c=horizon-0.1;

A_set = [eye(length(state));-eye(length(state))];
set = Polyhedron(state'-nominal_state');

for i=1:horizon
    u=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i);
    nominal_state = system_equation_nominal(nominal_state,u);
    set = Ak_sys*set;
    % Check unlikely case
    if length(fis)>1
        out = evalfis(fis{2},abs(nominal_state));
        b_set = [0.1*out;out;0.1*out;out];
        unlikely_set = Polyhedron(A_set,b_set)+set;
        if ~((nominal_state+unlikely_set)<X_set_full{2})
            break
        end
    end

    % Check normal case
    out = evalfis(fis{1},abs(nominal_state));
    b_set = [0.1*out;out;0.1*out;out];
    set = (Polyhedron(A_set,b_set)+Ak_sys*set);
    
    if ~((nominal_state+set)<X_set_full{1})
        break
    end
    c=c-1;
end
end