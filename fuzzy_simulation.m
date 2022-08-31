warning('off','all')
close all
%% Simulation
Number_of_iterations=25;
state=[20;0];
previous_solutions=[];
nominal_state=state;
state_history =zeros(Number_of_iterations,number_of_states);
nominal_state_history =zeros(Number_of_iterations,number_of_states);
X_set_unlikely = X_set_full+Polyhedron([0,0.2;0,-0.2]);

if length(fis_set)==2
    X_set_unlikely = X_set_full+Polyhedron([0,0.2;0,-0.2]);
    X_set_set = {X_set_full,X_set_unlikely};
    plot(X_set_unlikely,'color','r')
    hold on
    plot(X_set_full,'color','y')
else 
    plot(X_set_full,'color','r')
    X_set_set = {X_set_full};
    hold on
end


%plot(X_set_nominal,'color','g')
xlabel('position')
ylabel('velocity')
nominal_state_history(1,:)=nominal_state;
state_history(1,:)=state;

for i=1:Number_of_iterations
    nominal_state=state;
    %b_inequ = b_inequ_function(nominal_state);
    cost_function = @(U) (F*state+H*U')'*Q_full*(F*state+H*U') +U*R_full*U';
    tic
    [u,previous_solutions] = get_inputs_FLMPC(fis_set,Ak_sys,cost_function,X_set_set,horizon,number_inputs,...
        A_inputs,b_inputs,system_equation_nominal,state,nominal_state,previous_solutions,-2,2);
    toc
    %Hes=H'*Q_full*H+R_full;
    %grad=2*nominal_state'*F'*Q_full*H;
    %b_eq=b_eq_function(nominal_state);
    %[inputs,~,exitflag,message]=quadprog(Hes,grad,A_inequ,b_inequ,A_equ,b_eq);

%     if isempty(inputs)
%         disp(0);
%     end
%     u = inputs(1:number_inputs);
    u_hat = -K*(state-nominal_state);
    
    state = system_function(state,u+u_hat);
    
    

    nominal_state=A_sys*nominal_state+B_sys*u;
    state_history(i+1,:)=state;
    nominal_state_history(i+1,:)=nominal_state;
    plot(nominal_state_history(1:i+1,1),nominal_state_history(1:i+1,2),'b*-');
    plot(state_history(1:i+1,1),state_history(1:i+1,2),'k*-');
    if i==1

        plot(0,0,'g*')
        if length(fis_set)==2
            legend('feasible set','main feasible set','nominal state trajectory','real state trajectory','origin','AutoUpdate','off')
        else
            legend('feasible set','nominal state trajectory','real state trajectory','origin','AutoUpdate','off')
        end
    end

    pause(0.001) 
    saveas(gcf,'Barchart.png')
end

hold on