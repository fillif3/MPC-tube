function state=nominal_discrete_model(state,input,time_step,a,b,friction,Fzf,...
    Fzr,m,Iz,Cr,psi_d)

p=Iz/(m*b);

der=zeros(6,1);
der(1)=2*(a+b)*friction*Fzf*input(2)/(m*b)-state(3)*state(2); %There is an error in the paper
der(2)=2*friction*Fzf*input(1)/m+2*friction*Fzr*input(3)/m+state(3)*state(1)-state(3)^2*p;
der(3)=2*a*friction*Fzf*input(2)/Iz-2*b*Cr*state(1)/(Iz*state(2))+2*b*Cr*(b+p)*state(3)/(Iz*state(2));
der(4)=state(3)-psi_d;
der(5)=state(1)+state(2)*state(4);
der(6)=state(2);
state=state+time_step*der;
end