function plot_elipse(a,b,x0,y0)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y)
xlim([0 100])
ylim([-5 5])
end