%% Prepare parameters 
% Physical parameters
max_vel=0.22;
max_rot_vel = 2.84;


% MPC parameters

time_step=0.5;
horizon=10;
number_of_inputs=2;
nominal_velocity=0.1;
Q=diag([0,10,1]);
R=diag([1,0.1]);
lambda=10^5;


% Constraints parameters

max_y=5;
max_theta=pi/6;

% Simulation parameters

number_of_iterations=100;
%% Simulation
% Starting

state=zeros(5,1);
state(1)=4;
state(4)=nominal_velocity;


obstacles={[9,1.2,10,1],[9,1.2,30,-1]};

history_of_state=zeros(5,number_of_iterations);

model_simulation= @(state,input,ts) nominal_discrete_model(state,input,time_step);

nonlinear_constraint = @(state) safe_error(state,max_y,max_theta,obstacles);

% for i=1:number_of_iterations
%     input = nominal_mpc(state,model_simulation,Q,R,nominal_velocity,horizon,...
%     lambda,nonlinear_constraint,[max_vel;max_rot_vel]);
%     state= model_simulation(state,input);
%     noise=(rand(6,1)*2-1).*noise_bounds*time_step;
%     state=state+noise;
% 
%     history_of_state(:,i)=state;
%     previous_input=input;
% end
% 
% plot(history_of_state(1,:),history_of_state(2,:))
% hold on
% for i=1:length(obstacles)
%     obs = obstacles{i};
%     plot_elipse(obs(1),obs(2),obs(3),obs(4));
% end

nx = 5;
ny = 3;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

nlobj.Ts = 0.05;
nlobj.PredictionHorizon = 16;
nlobj.ControlHorizon = 16;

nlobj.Model.StateFcn = model_simulation;
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = @(x,u,Ts) [x(2); x(3);x(4)];

%nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));
%nlobj.Optimization.ReplaceStandardCost = true;

max_input=[max_y,max_theta,max_vel,max_rot_vel];
for ct = 1:4
    nlobj.States(ct).Min = -max_input(ct);
    nlobj.States(ct).Max = max_input(ct);
end
% 
nlobj.Weights.OutputVariables=[10,1,1000];
nlobj.Weights.ManipulatedVariables=ones(1,2);


nonlinear_constraint = @(X,U,e,data,params) CustomIneqConFcn(X,data,obstacles);

nlobj.Optimization.CustomIneqConFcn=nonlinear_constraint;

nloptions = nlmpcmoveopt;
nloptions.Parameters = {time_step};


validateFcns(nlobj, state, [0,0]', [], {0.05});
[~,~,info] = nlmpcmove(nlobj,state,[0;0],[0,0,nominal_velocity],[],nloptions);
u = info.MVopt(1,:);
previous_input=[0;0];
for i=1:number_of_iterations
    [~,~,info] = nlmpcmove(nlobj,state,previous_input,[0,0,nominal_velocity],[],nloptions);
    u = info.MVopt(1,:)';
    state= model_simulation(state,u,time_step);
    
    %noise=(rand(6,1)*2-1).*noise_bounds*time_step;
    %state=state+noise;

    history_of_state(:,i)=state;
    previous_input=u;
end

plot(history_of_state(1,:),history_of_state(2,:))
hold on
for i=1:length(obstacles)
    obs = obstacles{i};
    plot_elipse(obs(1),obs(2),obs(3),obs(4));
end
