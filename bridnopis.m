% val=zeros(21,1);
% k=0;
% for dist=0:0.5:10
%     k=k+1;
%     current_var=[0.1;1.3]*0.4/(1+dist/5);
%     %current_var=[0.1;1.3]*0.4;
%     max_res=0;
%     rests=zeros(1,100000);
%     for i=1:100000
%         res=normrnd([0;0],current_var);
%         rests(i)=res(2);
%     end
%     r=sort(rests);
%     val(k)=r(99000);
% end
val_full=[flip(val);val];
x=linspace(0,20,length(val_full));
opt = genfisOptions('GridPartition');
opt.NumMembershipFunctions = 3;
opt.InputMembershipFunctionType = "trimf";
    fis = genfis(x',val_full,opt);
[in,out,rule] = getTunableSettings(fis);
fisout = tunefis(fis,[in;out],x',val_full,tunefisOptions("Method","anfis"));