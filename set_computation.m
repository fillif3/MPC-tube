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
p=Iz/(m*b);
%L=0.1;

% MPC parameters

time_step=0.05;
horizon=16;
number_of_inputs=3;
nominal_velocity=7;

Q=diag([0.1,0.1,1,1,5,0.1]);
R=eye(3)/(10^10);
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

%% Set computation

A=zeros(6);
A(1,3)=-nominal_velocity;
A(3,1)=-2*b*Calpha/(Iz*nominal_velocity);
A(3,3)=2*b*Calpha*(p+b)/(Iz*nominal_velocity);
A(4,3)=1;
A(5,1)=1;
A(5,4)=nominal_velocity;
A(6,2)=nominal_velocity;

B=zeros(6,3);
B(1,2)=2*friction*force/m*(1+a/b);
B(2,1)=2*friction*force/m;
B(2,3)=B(2,1);
B(3,2)=2*a*friction*force/Iz;

[K,S,e] = lqrd(A,B,Q,R,0.05); % u=-kx

A=A*time_step+eye(6);
B=B*time_step;

%Ak = A-B*K;

% %% lateral  set computation
% 
% A_lateral = [A(2,2),A(2,6);A(6,2),A(6,6)];
% B_lateral=[B(2,1),B(2,3);B(6,1),B(6,3)];
% K_lateral = [K(1,2),K(1,6);K(3,2),K(3,6)];
% 
% Ak_lateral = A_lateral-B_lateral*K_lateral;
% 
% Noise_set_orignal_A_lateral=[eye(2);-eye(2)];
% Noise_set_orignal_b_lateral = [noise_bounds(2);noise_bounds(6);noise_bounds(2);noise_bounds(6)]*time_step;
% Noise_set_orignal_lateral = Polyhedron(Noise_set_orignal_A_lateral,Noise_set_orignal_b_lateral);
% 
% previous_omega_lateral = Noise_set_orignal_lateral;
% i=1;
% while true
%     i=i+1;
%     B_set_lateral=get_nonlinear_bound(previous_omega_lateral,L*time_step,2);
%     W_lateral= Noise_set_orignal_lateral+B_set_lateral;
%     omega_lateral = Ak_lateral*previous_omega_lateral+W_lateral;
%     if omega_lateral<=previous_omega_lateral
%        break
%     end
%     %if (~(omega_lateral>=previous_omega_lateral))
%     %    disp('jjj')
%     %end
%     %omega_lateral = Union([previous_omega_lateral,omega_lateral]);
%     previous_omega_lateral=omega_lateral;
%     if i==20
%         disp(i);
%     end
% end
% 
% plot(omega_lateral)
% hold on
% plot(previous_omega_lateral)

%% lateral  set computation

A_lateral = [A(1,1),A(1,3),A(1,4),A(1,5);
    A(3,1),A(3,3),A(3,4),A(3,5);
    A(4,1),A(4,3),A(4,4),A(4,5);
    A(5,1),A(5,3),A(5,4),A(5,5);];
B_lateral=[B(1,2);B(3,2);B(4,2);B(5,2)];
K_lateral = [K(2,1),K(2,3),K(2,4),K(2,5)];

Ak_lateral = A_lateral-B_lateral*K_lateral;

Noise_set_orignal_A_lateral=[eye(4);-eye(4)];
Noise_set_orignal_b_lateral = [noise_bounds([1,3,4,5]);noise_bounds([1,3,4,5])]*time_step;
Noise_set_orignal_lateral = Polyhedron(Noise_set_orignal_A_lateral,Noise_set_orignal_b_lateral);

previous_omega_lateral = Noise_set_orignal_lateral;
i=1;
while true
    i=i+1;
    %B_set_lateral=get_nonlinear_bound(previous_omega_lateral,L*time_step,4);
    %W_lateral= Noise_set_orignal_lateral+B_set_lateral;
    omega_lateral = Ak_lateral*previous_omega_lateral+Noise_set_orignal_lateral;
    if omega_lateral<=previous_omega_lateral
       break
    end
    %if (~(omega_lateral>=previous_omega_lateral))
    %    disp('jjj')
    %end
    %omega_lateral = Union([previous_omega_lateral,omega_lateral]);
    previous_omega_lateral=omega_lateral;
    if i==20
        disp(i);
    end
end

plot(omega_lateral)
hold on
plot(previous_omega_lateral)