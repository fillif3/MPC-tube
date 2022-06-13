clc
clear
rng("default")
%% Prepare parameters 
% Physical parameters

L=0.1;

% MPC parameters

number_of_inputs=1;


% Constraints parameters
A_sys = [1,1;0,1];%[1,Ts;0,1];
B_sys = [1;1];%[0;Ts];
number_of_states=length(A_sys);
[~,number_inputs]=size(B_sys);

Au=[eye(number_inputs);-eye(number_inputs)];
bu=ones(number_inputs*2,1)*6;
U_set_full = Polyhedron(Au,bu);

A=[1,1;1,0;-1,-1;-1,0];
b=ones(4,1)*10;
X_set_full = Polyhedron(A,b);

% Simulation parameters


%% Set computation



K=[1.17,1.03]*0.9;

Ak = A_sys-B_sys*K;
noise_bounds = [1;1]*0.7;
W_set=get_noise_set_from_bounds(noise_bounds);


previous_omega_lateral = W_set;
i=1;
while true
    i=i+1;
    B_set_lateral=get_nonlinear_bound(previous_omega_lateral,L,2);
    W_lateral= W_set+B_set_lateral;
    omega_lateral = Ak*previous_omega_lateral+W_lateral;
    if omega_lateral<=previous_omega_lateral
       break
    end
    %if (~(omega_lateral>=previous_omega_lateral))
    %    disp('jjj')
    %end
    %omega_lateral = Union([previous_omega_lateral,omega_lateral]);
    previous_omega_lateral=omega_lateral;
    if i==20
        disp(i);
    end
end

tube=previous_omega_lateral;
X_set_nominal=X_set_full-tube;
U_set_nominal= U_set_full-K*tube;

% Nominal MPC
Q = diag([0.1,1]);
R=eye(number_inputs);
horizon=20;
[F,H]=get_prediction_matrices(A_sys,B_sys,horizon);
[A_states,b_states] = get_prediction_constraints(X_set_nominal.A,X_set_nominal.b,horizon);
A_states_final = A_states*H;
A_equ =H((end-(number_of_states-1)):end,:);
b_eq_function = @(x0) -[F((end-(number_of_states-1)):end,:)*x0];
[A_inputs,b_inputs] = get_prediction_constraints(U_set_nominal.A,U_set_nominal.b,horizon);
A_inequ=[A_states_final;A_inputs];
b_inequ_function = @(x0) [b_states-A_states*F*x0;b_inputs];



[Q_full,R_full]=get_full_weight_matrices(Q,R,horizon);

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