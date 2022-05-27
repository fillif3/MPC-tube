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


% MPC parameters

time_step=0.05;
horizon=16;
number_of_inputs=3;
nominal_velocity=7;
Q=diag([1,0.1,1,1,5,0]);
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

%% Simulation
% Starting

state=zeros(6,1);
state(2)=7;
state(6)=15;
previous_input=zeros(3,1);
obstacles={[9,1.2,30,1],[9,1.2,70,-1]};

history_of_state=zeros(6,number_of_iterations);

model_simulation= @(state,input,time_step) nominal_discrete_model(state,input,time_step,a,b,friction,force,...
    force,m,Iz,Calpha,0);
model= @(state,input,time_step) nominal_discrete_simplfied_model(state,input,time_step,a,b,friction,force,...
    force,m,Iz,Calpha,0);
nonlinear_constraint = @(X,U,e,data,params) CustomIneqConFcn(X,data,obstacles,error_y_max,alpha_max,a,b);

for i=1:number_of_iterations
    input = nominal_mpc(state,trajectory(i:(i+horizon)),obstacles,model,Q,R,S,horizon,...
    lambda,number_of_inputs,error_y_max,a,b,alpha_max,previous_input,max_input);
    state= model(state,input,time_step);
    noise=(rand(6,1)*2-1).*noise_bounds*time_step;
    state=state+noise;

    history_of_state(:,i)=state;
    previous_input=input;
end

plot(history_of_state(6,:),history_of_state(5,:))
hold on
for i=1:length(obstacles)
    obs = obstacles{i};
    plot_elipse(obs(1),obs(2),obs(3),obs(4));
end

% nx = 6;
% ny = 4;
% nu = 3;
% nlobj = nlmpc(nx,ny,nu);
% 
% nlobj.Ts = 0.05;
% nlobj.PredictionHorizon = 16;
% nlobj.ControlHorizon = 16;
% 
% nlobj.Model.StateFcn = model;
% nlobj.Model.IsContinuousTime = false;
% nlobj.Model.NumberOfParameters = 1;
% nlobj.Model.OutputFcn = @(x,u,Ts) [x(2); x(3);x(4); x(5)];
% 
% %nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));
% %nlobj.Optimization.ReplaceStandardCost = true;
% 
% for ct = 1:nu
%     nlobj.MV(ct).Min = -max_input;
%     nlobj.MV(ct).Max = max_input;
% end
% 
% nlobj.Weights.OutputVariables=[Q(2,2)*1000,Q(3,3),Q(4,4),Q(4,4)];
% nlobj.Weights.ManipulatedVariablesRate=ones(1,3);
% nlobj.Weights.ManipulatedVariables=ones(1,3);
% 
% nlobj.Optimization.CustomIneqConFcn=nonlinear_constraint;
% 
% nloptions = nlmpcmoveopt;
% nloptions.Parameters = {time_step};
% 
% 
% %validateFcns(nlobj, state, [0,0,0]', [], {0.05});
% [~,~,info] = nlmpcmove(nlobj,state,[0;0;0],[7,0,0,0],[],nloptions);
% u = info.MVopt(1,:);
% 
% for i=1:number_of_iterations
%     [~,~,info] = nlmpcmove(nlobj,state,previous_input,[7,0,0,0],[],nloptions);
%     u = info.MVopt(1,:)';
%     state= model(state,u,time_step);
%     noise=(rand(6,1)*2-1).*noise_bounds*time_step;
%     %state=state+noise;
% 
%     history_of_state(:,i)=state;
%     previous_input=u;
% end

% plot(history_of_state(6,:),history_of_state(5,:))
% hold on
% for i=1:length(obstacles)
%     obs = obstacles{i};
%     plot_elipse(obs(1),obs(2),obs(3),obs(4));
% end
