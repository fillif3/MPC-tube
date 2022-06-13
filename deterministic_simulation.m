A_sys = [1,1;0,1];%[1,Ts;0,1];
B_sys = [1;1];%[0;Ts];
horizon=20;
A=[1,1;1,0;-1,-1;-1,0];
b=ones(4,1)*10;
X_set_full = Polyhedron(A,b);
number_inputs=1;
number_of_states=2;
noise_bounds = [1;1]*0.7;
Au=[eye(number_inputs);-eye(number_inputs)];
bu=ones(number_inputs*2,1)*6;
U_set_full = Polyhedron(Au,bu);
[F,H]=get_prediction_matrices(A_sys,B_sys,horizon);
[A_states,b_states] = get_prediction_constraints(X_set_full.A,X_set_full.b,horizon);
A_states_final = A_states*H;
A_equ =H((end-(number_of_states-1)):end,:);
b_eq_function = @(x0) -[F((end-(number_of_states-1)):end,:)*x0];
[A_inputs,b_inputs] = get_prediction_constraints(U_set_full.A,U_set_full.b,horizon);
A_inequ=[A_states_final;A_inputs];
b_inequ_function = @(x0) [b_states-A_states*F*x0;b_inputs];

Q = diag([0.1,1]);
R=eye(number_inputs);

[Q_full,R_full]=get_full_weight_matrices(Q,R,horizon);

%% Simulation
Number_of_iterations=100;
state=[0;0];
state_history =zeros(Number_of_iterations,number_of_states);
plot(X_set_full)
hold on

for i=1:Number_of_iterations
    b_inequ = b_inequ_function(state);
    Hes=H'*Q_full*H+R_full;
    grad=2*state'*F'*Q_full*H;
    b_eq=b_eq_function(state);
    [inputs,~,exitflag,message]=quadprog(Hes,grad,A_inequ,b_inequ,A_equ,b_eq);
    if isempty(inputs)
        disp(0);
    end
    u = inputs(1:number_inputs);

    
    state = A_sys*state+B_sys*(u)+(rand(size(noise_bounds))*2.*noise_bounds-(noise_bounds));

    state_history(i,:)=state;

    


end
plot(tube,'color','white')
hl2=plot(state_history(:,1),state_history(:,2),'b');
plot(0,0,'g*')
xlabel('x1')
ylabel('x2')
legend('Feasible set','trajectory')
