function fval = hw(c,W)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
options = optimoptions('linprog','Display','off');
[~,fval]=linprog(-c,W.A,W.b,[],[],[],[],options);
fval=-fval;
end