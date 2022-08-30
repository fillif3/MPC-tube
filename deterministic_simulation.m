%% Simulation
Number_of_iterations=300;
state=[20;0];
state_history =zeros(Number_of_iterations,number_of_states);
plot(X_set_full)
hold on

for i=1:Number_of_iterations
    b_inequ = b_inequ_function(state);
    Hes=H'*Q_full*H+R_full;
    grad=2*state'*F'*Q_full*H;
    b_eq=b_eq_function(state);
    [inputs,~,exitflag,message]=quadprog(Hes,grad,A_inequ,b_inequ,[],[]);
    if isempty(inputs)
        disp(0);
    end
    u = inputs(1:number_inputs);

    
    state =system_function(state,u);

    state_history(i,:)=state;

    


end
%plot(tube,'color','white')
hl2=plot(state_history(:,1),state_history(:,2),'b');
plot(0,0,'g*')
xlabel('x1')
ylabel('x2')
legend('Feasible set','trajectory')
