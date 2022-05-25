function trajectory = get_trajectory(number_of_iterations)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
trajectory=cell(number_of_iterations,1);
state=zeros(6,1);
state(2)=7;
for i=1:number_of_iterations
    trajectory{i}=state;
end