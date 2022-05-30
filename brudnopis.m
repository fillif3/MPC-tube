leng=10;
x=zeros(leng,1);
y=zeros(leng,1);
state=[0,0,0];

for i=1:leng
    state= nominal_discrete_model(state,[-1,1],0.1);
    x(i)=state(1);
    y(i)=state(2);
end
plot(x,y)