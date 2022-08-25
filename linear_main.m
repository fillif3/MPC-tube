%clear
%clc
rng("default")
%% System definition

A_sys = [1,1;0,1];%[1,Ts;0,1];
B_sys = [0;1];%[0;Ts];
number_of_states=length(A_sys);
[~,number_inputs]=size(B_sys);
Ts=1;
% Noise
noise_bounds = [0.1;1.3]*0.5;
W_set=get_noise_set_from_bounds(noise_bounds*Ts);

% System function
%system_function =@(x,u) A_sys*x+B_sys*u+(rand(size(noise_bounds))*2.*noise_bounds-(noise_bounds));
highest_variance_position=10;
system_function=@(x,u) biased_system(A_sys,B_sys,x,u,highest_variance_position,noise_bounds/2);
system_equation_nominal=@(x,u) A_sys*x+B_sys*u;

% Constraints
A=[0,1;1,0;0,-1;-1,0];
b=[1.5;30;1.5;30];
X_set_full = Polyhedron(A,b);
Au=[eye(number_inputs);-eye(number_inputs)];
bu=ones(number_inputs*2,1)*6;
U_set_full = Polyhedron(Au,bu);

%Noise



%% Internal controller

Q_internal=[1,0;0,1];
R_internal=0.1;
[K,~,~] = dlqr(A_sys,B_sys,Q_internal,R_internal,0);

%K=[1.17,1.03]*0.5;
Ak_sys = A_sys-B_sys*K;


tube=get_minimum_linear_robust_invariant_set(Ak_sys,W_set,10^(-2),1);
X_set_nominal=X_set_full-tube;
U_set_nominal= U_set_full-K*tube;
figure
plot(X_set_full)
hold on
plot(X_set_nominal,'color','g')
plot(tube,'color','white')
xlabel('x1')
ylabel('x2')
legend('Feasible set','Reduced Feasible set','Tube')
figure
plot(U_set_nominal)
xlabel('u1')
legend('Reduced input set')

%% Nominal MPC
horizon=5;

[F,H]=get_prediction_matrices(A_sys,B_sys,horizon);
[A_states,b_states] = get_prediction_constraints(X_set_nominal.A,X_set_nominal.b,horizon);
A_states_final = A_states*H;
A_equ =H((end-(number_of_states-1)):end,:);
b_eq_function = @(x0) -[F((end-(number_of_states-1)):end,:)*x0];
[A_inputs,b_inputs] = get_prediction_constraints(U_set_nominal.A,U_set_nominal.b,horizon);
A_inequ=[A_states_final;A_inputs];
b_inequ_function = @(x0) [b_states-A_states*F*x0;b_inputs];

Q = diag([0.1,1]);
R=eye(number_inputs);

[Q_full,R_full]=get_full_weight_matrices(Q,R,horizon);

