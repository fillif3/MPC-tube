%% Pameters 
% sys
A_sys = [1,1;0,1];%[1,Ts;0,1];
B_sys = [1;1];%[0;Ts];
number_inputs=1;
number_of_states=2; 
noise_bounds = [1;1]*0;

% Internal Contorller
K=[1.17,1.03];
% Nominal MPC
horizon=20;
[F,H]=get_prediction_matrices(A_sys,B_sys,horizon);
Q = diag([0.1,1]);
R=eye(number_inputs);
[Q_full,R_full]=get_full_weight_matrices(Q,R,horizon);

% constraints
A=[1,1;1,0;-1,-1;-1,0];
b=ones(4,1)*10;
X_set_full = Polyhedron(A,b);
Au=[eye(number_inputs);-eye(number_inputs)];
bu=ones(number_inputs*2,1)*6;
U_set_full = Polyhedron(Au,bu);
[A_states,b_states] = get_prediction_constraints(X_set_full.A,X_set_full.b,horizon);
% Add terminal contraints
A_states_final = A_states*H;
A_equ =H((end-(number_of_states-1)):end,:);
b_eq_function = @(x0) -[F((end-(number_of_states-1)):end,:)*x0];
[A_inputs,b_inputs] = get_prediction_constraints(U_set_full.A,U_set_full.b,horizon);
A_inequ=[A_states_final;A_inputs];
b_inequ_function = @(x0) [b_states-A_states*F*x0;b_inputs];
