warning('off','all')
%% Simulation
Number_of_iterations=100;
state=[20;0];
nominal_state=state;
state_history =zeros(Number_of_iterations,number_of_states);
nominal_state_history =zeros(Number_of_iterations,number_of_states);
plot(X_set_full)
hold on
%plot(X_set_nominal,'color','g')
xlabel('position')
ylabel('velocity')
for i=1:Number_of_iterations
    %b_inequ = b_inequ_function(nominal_state);
    cost_function = @(U) (F*state+H*U')'*Q_full*(F*state+H*U') +U*R_full*U';
    u = get_inputs_FLMPC(fisout,Ak_sys,cost_function,X_set_full,horizon,number_inputs,...
        A_inputs,b_inputs,system_equation_nominal,state,nominal_state);
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
    state_history(i,:)=state;
    nominal_state_history(i,:)=nominal_state;
    plot(nominal_state_history(1:i,1),nominal_state_history(1:i,2),'b*-');
    plot(state_history(1:i,1),state_history(1:i,2),'k*-');


    pause(0.1) 
end

hold on