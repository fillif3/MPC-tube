%% Set computation

Ak = A_sys-B_sys*K;
W_set=get_noise_set_from_bounds(noise_bounds);


previous_omega_lateral = W_set;
i=1;
while true
    i=i+1;
    B_set_lateral=get_nonlinear_bound(previous_omega_lateral,L,2);
    W_lateral= W_set+B_set_lateral;
    omega_lateral = Ak*previous_omega_lateral+W_lateral;
    if omega_lateral<=previous_omega_lateral
       break
    end
    previous_omega_lateral=omega_lateral;
    if i==20
        disp(i);
    end
end


X_set_nominal=X_set_full-tube;
U_set_nominal= U_set_full-K*tube;
figure
plot(X_set_full)
hold on
plot(X_set_nominal,'color','g')
plot(tube,'color','white')
xlabel('x1')
ylabel('x2')
legend('Feasible set','Reduced Feasible set','Tube')
figure
plot(U_set_nominal)
xlabel('u1')
legend('Reduced input set')