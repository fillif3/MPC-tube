function population = create_pso_population(previous_solution,lb,ub,number_of_partciles)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
random_particles= floor(number_of_partciles/2);
non_random_particles = number_of_partciles- random_particles;
number_of_altered_partclies = floor(non_random_particles/2);
population = zeros(length(previous_solution),number_of_partciles);
population(1:(length(previous_solution)-1),1:non_random_particles)=repmat(previous_solution(2:end)',[1,non_random_particles]);
population(end,1:non_random_particles)=(ub-lb)*rand([1,non_random_particles])+lb;
population(1:4,1:number_of_altered_partclies)=population(1:4,1:number_of_altered_partclies)+((ub-lb)*rand([4,number_of_altered_partclies])+lb)/10;
population(:,(non_random_particles+1):end) = (ub-lb)*rand([length(previous_solution),random_particles])+lb;

end