%% Prepare parameters 
% Physical parameters
a=0.3;
b=0.3;
force=4000;
m=2050;
Iz=3344;
friction=0.5;
Calpha=65000;


% MPC parameters

time_step=0.05;
horizon=999;
number_of_inputs=3;

% Constraints parameters

error_y_max=3.5;
input_max=0.5;
alpha_max=4/180*pi;
Q=diag([0,0.1,1,1,5,0]);
R=eye(3);
S=eye(3);
lambda=10^5;

% Simulation parameters

number_of_iterations=1000;
trajectory=get_trajectory(number_of_iterations);

%% Simulation
% Starting

state=zeros(6,1);
state(2)=7;
previous_input=zeros(3,1);
obstacles={[9,1.2,30,1],[9,1.2,70,-1]};

history_of_state=zeros(6,number_of_iterations);

model= @(input,state) nominal_discrete_model(state,input,time_step,a,b,friction,force,...
    force,m,Iz,Calpha,0);

for i=1:number_of_iterations
    input = nominal_mpc(state,trajectory(i:(i+horizon)),obstacles,model,Q,R,S,horizon,...
    lambda,number_of_inputs,error_y_max,a,b,alpha_max,previous_input);
    state= model(input,state);


    history_of_state(:,i)=state;
    previous_input=input;
end


