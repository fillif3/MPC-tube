%% Simulation
Number_of_iterations=100;
state=[-4;4];
nominal_state=state;
state_history =zeros(Number_of_iterations,number_of_states);
plot(X_set_full)
hold on
plot(X_set_nominal,'color','g')

for i=1:Number_of_iterations
    b_inequ = b_inequ_function(nominal_state);
    Hes=H'*Q_full*H+R_full;
    grad=2*nominal_state'*F'*Q_full*H;
    b_eq=b_eq_function(nominal_state);
    [inputs,~,exitflag,message]=quadprog(Hes,grad,A_inequ,b_inequ);
    if isempty(inputs)
        disp(0);
    end
    u = inputs(1:number_inputs);
    u_hat = -K*(state-nominal_state);
    
    state = A_sys*state+B_sys*(u+u_hat)+(rand(size(noise_bounds))*2.*noise_bounds-(noise_bounds))+0.1*sin(state);

    nominal_state=A_sys*nominal_state+B_sys*u;
    state_history(i,:)=state;
    try
        delete( hl1 );
        delete( hl2 );
    catch
        
    end
    hl1=plot(tube+nominal_state,'color','white');
    hl2=plot(state_history(max(1,(i-3)):i,1),state_history(max(1,(i-3)):i,2),'b');
    plot(0,0,'r*')
    if i==1
        xlabel('x1')
        ylabel('x2')
        legend({'Feasible set','Reduced Feasible set','Tube','trajectory'},'AutoUpdate','off')
        
    end
    pause(1) 
end

hold on