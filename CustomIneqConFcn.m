function c = CustomIneqConFcn(X,data,obstacles)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

p = data.PredictionHorizon;
c=zeros(p,1);
for i=2:p+1
    c(p-1)=max(safe_error(X(i,:)',obstacles));
end
end