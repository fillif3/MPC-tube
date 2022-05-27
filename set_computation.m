%% Prepare parameters 
% Physical parameters
a=0.3;
b=0.3;
force=4000;
m=2050;
Iz=3344;
friction=0.5;
Calpha=65000;
noise_bounds = [0.2,0.2,0.2,0.005,0.05,0.05]';
p=Iz/(m*b);

% MPC parameters

time_step=0.05;
horizon=16;
number_of_inputs=3;
nominal_velocity=7;

Q=diag([0.00001,0.1,1,1,5,0.00001]);
R=eye(3);
S=eye(3);
lambda=10^5;


% Constraints parameters

error_y_max=3.5;
input_max=0.5;
alpha_max=4/180*pi;
max_input=0.5;


% Simulation parameters

number_of_iterations=75;
trajectory=get_trajectory(number_of_iterations+horizon);

%% Set computation

A=zeros(6);
A(1,3)=-nominal_velocity;
A(3,1)=-2*b*Calpha/(Iz*nominal_velocity);
A(3,3)=2*b*Calpha*(p+b)/(Iz*nominal_velocity);
A(4,3)=1;
A(5,1)=1;
A(5,4)=nominal_velocity;
A(6,2)=nominal_velocity;

B=zeros(6,3);
B(1,2)=2*friction*force/m*(1+a/b);
B(2,1)=2*friction*force/m;
B(2,3)=B(2,1);
B(3,2)=2*a*friction*force/Iz;

[K,S,e] = lqrd(A,B,Q,R,0.05); % u=-kx

A=A*time_step+eye(6);
B=B*time_step;

Ak = A-B*K;