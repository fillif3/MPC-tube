a=0.3;
b=0.3;
force=4000;
m=2050;
Iz=3344;
friction=0.5;
Calpha=65000;

max_y_error=3.5;
input_max=0.5;
max_alfa=4/180*pi;
max_input=0.5;
model2 = @(x) g(x,Iz,m,b,7,Calpha);
model = @(x) -find_len_of_gradient_constant(x,Calpha,b,Iz,m,7);
nonlinear_constraint_function = @(x) [safe_error(x,[],max_y_error,max_alfa,a,b);...
    safe_error(x+0.001,[],max_y_error,max_alfa,a,b)];
nonloc = @(x) wrapper(x,nonlinear_constraint_function);
A=zeros(4,4);
A(1,2)=1;
A(2,2)=-1;
A(3,4)=1;
A(4,4)=-1;

% A(5,8)=1;
% A(6,8)=-1;
% A(7,10)=1;
% A(8,10)=-1;
% A(9,6)=1;
% A(9,12)=-1;
b=zeros(4,1);
b(1)=8;
b(2)=6;
b(3)=pi/4;
b(4)=-0.1;
% b(5)=10;
% b(6)=4;
% b(7)=pi/4;
% b(8)=-pi/4;
% b(9)=-0.1;


[L,fval,flag,message] = ga(model,4,A,b,[],[],[],[],nonloc);
disp(fval) %solution -> 8.5627
function grad= find_len_of_gradient_constant(x,Cr,b,I,m,v)
p=I/(m*b);
alfa_r=(x(1)-b*x(3))/x(2);
x1=x(1);
x2=x(2);
x3=x(3);
x4=x(4);
C=2*Cr*b*alfa_r;
denominative=sqrt((x1^2*C^2*(x2-x3*(b+p))^2)/(I^2*v^2*(x2+v)^2)+x2^2*x4^2+x3^2*(x1-x3*p)^2+x3^2);
x1_der = (C^2*x1*(x2-x3*(b+p))^2+(x3*I*v*(x2+v))^2*(x1-x3*p))/((I*v*(x2+v))^2);
x2_der = 2*x2*x4^2+(2*x1^2*C^2*(x2-x3*(b+p)))/(I^2*v^2*(x2+v)^2)-(2*x1*C*(x2-x3*(b+p)))^2/(I^2*v^2*(x2+v)^3);
x3_der = (x1^2*C^2*(b+p)*(x3*(b+p)-x2))/(I^2*v^2*(x2+v)^2)+x3*(x1-2*x3*p)*(x1-x3*p)+x3;
x4_der=x2^2*x4;
grad=norm([x1_der,x2_der,x3_der,x4_der])/denominative;


% https://www.wolframalpha.com/input?i2d=true&i=D%5BSqrt%5BPower%5Bg%2C2%5D%2BPower%5B%5C%2840%29g*e-Power%5Bg%2C2%5D*p%5C%2841%29%2C2%5D%2BDivide%5BPower%5B2*C*%5C%2840%29e*f-%5C%2840%29p%2Bb%5C%2841%29*g*e%5C%2841%29%2C2%5D%2CPower%5Bk*v%5C%2840%29v%2Bf%5C%2841%29%2C2%5D%5D%2BPower%5Bf*h%2C2%5D%5D%2Ce%5D
%https://www.wolframalpha.com/input?i2d=true&i=D%5BSqrt%5BPower%5Bg%2C2%5D%2BPower%5B%5C%2840%29g*e-Power%5Bg%2C2%5D*p%5C%2841%29%2C2%5D%2BDivide%5BPower%5B2*C*%5C%2840%29e*f-%5C%2840%29p%2Bb%5C%2841%29*g*e%5C%2841%29%2C2%5D%2CPower%5Bk*v%5C%2840%29v%2Bf%5C%2841%29%2C2%5D%5D%2BPower%5Bf*h%2C2%5D%5D%2Cf%5D
%https://www.wolframalpha.com/input?i2d=true&i=D%5BSqrt%5BPower%5Bg%2C2%5D%2BPower%5B%5C%2840%29g*e-Power%5Bg%2C2%5D*p%5C%2841%29%2C2%5D%2BDivide%5BPower%5B2*C*%5C%2840%29e*f-%5C%2840%29p%2Bb%5C%2841%29*g*e%5C%2841%29%2C2%5D%2CPower%5Bk*v%5C%2840%29v%2Bf%5C%2841%29%2C2%5D%5D%2BPower%5Bf*h%2C2%5D%5D%2Cg%5D
%https://www.wolframalpha.com/input?i2d=true&i=D%5BSqrt%5BPower%5Bg%2C2%5D%2BPower%5B%5C%2840%29g*e-Power%5Bg%2C2%5D*p%5C%2841%29%2C2%5D%2BDivide%5BPower%5B2*C*%5C%2840%29e*f-%5C%2840%29p%2Bb%5C%2841%29*g*e%5C%2841%29%2C2%5D%2CPower%5Bk*v%5C%2840%29v%2Bf%5C%2841%29%2C2%5D%5D%2BPower%5Bf*h%2C2%5D%5D%2Ch%5D
end

function out=g(x,Iz,m,b,nominal_velocity,Calpha)

p=Iz/(m*b);
alfa_r=(x(1)-b*x(3))/x(2);
x2_diff=(x(2)-nominal_velocity);
p=Iz/(m*b);
out=zeros(6,1);
out(1)=-x(3);
out(2)=x(3)*x(1)-x(3)^2*p;
out(3)=2*b*Calpha*alfa_r*(x(1)*x2_diff-(p+b)*x(3)*x2_diff)/(Iz*nominal_velocity*x(2));
out(5)=x2_diff*x(4);

end

function [c,ceq]=wrapper(x,func)
ceq=0;
c=func(x);
end