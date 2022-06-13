A=[1,0;0,1;-1,0;0,-1;1,1;-1,1];
b=[2,1,2,0,1.5,1.5]';
P=Polyhedron(A,b);
plot(P)
hold on
xlim([-2,2])
ylabel('u(x)')
xlabel('x')
A2= [0,-1;1,1;-1,2];
b2=[0,1,2]';
P2=Polyhedron(A2,b2);
plot(P2,'color','b')
