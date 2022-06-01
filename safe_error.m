function [errors] = safe_error(state,obstacles,max_y,max_theta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

number_of_obstacles=length(obstacles);
errors=zeros(1,1);
if number_of_obstacles>0
    errors(1)=-1000;
    for i=1:number_of_obstacles
        obstacle=obstacles{i};
        ey_obs=obstacle(4);
        s_obs=obstacle(3);
        
        a_obs=obstacle(1);
        b_obs=obstacle(2);
        errors(1) = max(1-((state(1)-s_obs)/a_obs)^2-((state(2)-ey_obs)/b_obs)^2,errors(1));
    end
end
errors(2)=-state(2)-max_y;
errors(3)=state(2)-max_y;
errors(4)=-state(3)-max_theta;
errors(5)=state(3)-max_theta;
end