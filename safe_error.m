function [errors] = safe_error(state,obstacles,max_y_error,max_alfa,a,b)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


errors=zeros(8,1);

%errors(1)=-state(5)-max_y_error;
%errors(2)=state(5)-max_y_error;
number_of_obstacles=length(obstacles);
if number_of_obstacles>0
    errors(3)=-1000;
    for i=1:number_of_obstacles
        obstacle=obstacles{i};
        ey_obs=obstacle(4);
        s_obs=obstacle(3);
        
        a_obs=obstacle(1);
        b_obs=obstacle(2);
        errors(3) = max(1-((state(6)-s_obs)/a_obs)^2-((state(5)-ey_obs)/b_obs)^2,errors(3));
    end
end
if state(2)==0
    errors(4:7)=ones(4,1)*(-1000);
else
    alfa_f=(state(1)+a*state(3))/state(2);
    alfa_r=(state(1)-b*state(3))/state(2);
    
    errors(4)=-alfa_f-max_alfa;
    errors(5)=alfa_f-max_alfa;
    
    errors(6)=-alfa_r-max_alfa;
    errors(7)=alfa_r-max_alfa;
end
% disp('----------------')
% disp(state)
% disp(errors)
end