rng("default")

%% System definition

A_sys = [1,1;0,1];%[1,Ts;0,1];
B_sys = [1;1];%[0;Ts];
number_of_states=length(A_sys);
[~,number_inputs]=size(B_sys);


A=[1,1;1,0;-1,-1;-1,0];
b=ones(4,1)*10;
X_set_full = Polyhedron(A,b);

Au=[eye(number_inputs);-eye(number_inputs)];
bu=ones(number_inputs*2,1)*6;
U_set_full = Polyhedron(Au,bu);
%plot(X_states_full)
Ts=1;




noise_bounds = [1;1]*0.7;
W_set=get_noise_set_from_bounds(noise_bounds*Ts);
%hold on
%plot(W_set)


%Q=diag([10,0.1]);
%R=1;
%[K,~,~]= dlqr(A_sys,B_sys,Q,R);

% Tube controller

K=[1.17,1.03]*0.8;
Ak_sys = A_sys-B_sys*K;


tube=get_minimum_linear_robust_invariant_set(Ak_sys,W_set,10^(-2));
X_set_nominal=X_set_full-tube;
U_set_nominal= U_set_full-K*tube;

%% Nominal MPC
horizon=20;

[F,H]=get_prediction_matrices(A_sys,B_sys,horizon);
[A_states,b_states] = get_prediction_constraints(X_set_nominal.A,X_set_nominal.b,horizon);
A_states_final = A_states*H;
A_equ =H((end-(number_of_states-1)):end,:);
b_eq_function = @(x0) [F((end-(number_of_states-1)):end,:)*x0];
[A_inputs,b_inputs] = get_prediction_constraints(U_set_nominal.A,U_set_nominal.b,horizon);
A_inequ=[A_states_final;A_inputs];
b_inequ_function = @(x0) [b_states-A_states*F*x0;b_inputs];

Q = diag([0.1,1]);
R=eye(number_inputs);

[Q_full,R_full]=get_full_weight_matrices(Q,R,horizon);

%% Simulation
Number_of_iterations=100;
state=[-4;5];
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
    [inputs,~,exitflag,message]=quadprog(Hes,grad,A_inequ,b_inequ,A_equ,b_eq);
    if isempty(inputs)
        disp(0);
    end
    u = inputs(1:number_inputs);
    u_hat = -K*(state-nominal_state);
    
    state = A_sys*state+B_sys*(u+u_hat)+(rand(size(noise_bounds))*2.*noise_bounds-(noise_bounds));

    nominal_state=A_sys*nominal_state+B_sys*u;
    state_history(i,:)=state;
    try
        delete( hl1 );
        delete( hl2 );
    catch
        
    end
    hl1=plot(tube+nominal_state,'color','white');
    hl2=plot(state_history(1:i,1),state_history(1:i,2));
    pause(2) 
end

hold on

