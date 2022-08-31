function [u,Population] = get_inputs_FLMPC(fis,Ak_sys,cost_function,X_set_full,horizon,...
    number_of_inputs,A,b,system_equation_nominal,state,nominal_state,previous_genration,lb,ub)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
population_size=40;
nonlocon = @(u) nonlocon_helper(u,fis,state,nominal_state,Ak_sys,...
    system_equation_nominal,X_set_full,horizon,number_of_inputs);
%options = optimoptions('ga','Display','off');
if isempty(previous_genration)
    options = optimoptions('ga','MaxGenerations',10,'PopulationSize',40);
else
    new_generation = zeros(population_size,horizon*number_of_inputs);
    new_generation(1:population_size/2,1:end-number_of_inputs) =previous_genration(1:population_size/2,number_of_inputs+1:end);
    new_generation(1:population_size/2,(horizon-1)*number_of_inputs+1:number_of_inputs*horizon)=2*rand([population_size/2,number_of_inputs])-1;
    new_generation((population_size/2+1):population_size,:)=(ub-lb)*rand(population_size/2,number_of_inputs*horizon)+lb;
    options = optimoptions('ga','MaxGenerations',6,'PopulationSize',population_size,'InitialPopulationMatrix',new_generation);

end
try
    [~,~,EXITFLAG,~,Population,Scores] = ga(cost_function,horizon*number_of_inputs,A,b,[],[],lb*ones(horizon*number_of_inputs),ub*ones(horizon*number_of_inputs),nonlocon,options);
catch
    qwe=1;
end
idexes_to_delete=[];
for i=1:population_size
    if nonlocon(Population(i,:))>0
        idexes_to_delete(end+1)=i;
    end
end
Population(idexes_to_delete,:)=[];
Scores(idexes_to_delete)=[];

[~,idx]=sort(Scores);
sorted_population = Population(idx,:);
u=sorted_population(1,1);
end

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

% for i=1:horizon
%     u=inputs((number_of_inputs*(i-1)+1):number_of_inputs*i);
%     nominal_state = system_equation_nominal(nominal_state,u);
%     set = Ak_sys*set;
%     % Check unlikely case
%     if length(fis)>2
%     out = evalfis(fisout_unlikely,abs(nominal_state));
%     b_set = [0.1*out;out;0.1*out;out];
%     unlikely_set = Polyhedron(A_set,b_set)+set;
%     if ~((unlikely_set+unlikely_set)<X_set_unlikely)
%         break
%     end
%     end
% 
%     % Check normal case
%     out = evalfis(fis{1},abs(nominal_state));
%     b_set = [0.1*out;out;0.1*out;out];
%     set = (Polyhedron(A_set,b_set)+Ak_sys*set);
%     
%     if ~((nominal_state+set)<X_set_full{1})
%         break
%     end
%     c=c-1;
% end
% end