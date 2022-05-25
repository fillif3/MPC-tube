function u = nominal_mpc2(state,trajectory,obstacles,model,Q,R,S,horizon,...
    lambda,number_of_inputs,max_y_error,a,b,max_alfa,previous_input,max_input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nonlinear_constraint_function = @(state) safe_error(state,obstacles,max_y_error,max_alfa,a,b);
cost_function= @(X,input,e,data) cost_following_trajectory(input,state,model,trajectory,...
    Q,R,S,lambda,horizon,number_of_inputs,previous_input,nonlinear_constraint_function);


nx = 6;
ny = 4;
nu = 3;
nlobj = nlmpc(nx,ny,nu);

nlobj.Ts = 0.05;
nlobj.PredictionHorizon = 16;
nlobj.ControlHorizon = 16;

nlobj.Model.StateFcn = model;
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = @(x,u,Ts) [x(2); x(3),x(3); x(5)];

%nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));
%nlobj.Optimization.ReplaceStandardCost = true;

for ct = 1:nu
    nlobj.MV(ct).Min = -max_input;
    nlobj.MV(ct).Max = max_input;
end

nlobj.Weigths.OutputVariables=[Q(2,2),Q(3,3),Q(4,4),Q(4,4)];
nlobj.Weigths.ManipulatedVariablesRate=ones(3,1);
nlobj.Weigths.ManipulatedVariables=ones(3,1);
    

end