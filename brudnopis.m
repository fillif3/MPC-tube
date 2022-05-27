A=[eye(2);-eye(2)];
A(4,1)=1;
b=ones(4,1);
P1=Polyhedron(A,b);
Aw=[eye(2);-eye(2)];
bw=ones(4,1)*0.1;
W=Polyhedron(Aw,bw);
%P2=Polyhedron(A,b);

%P3=P2-P1;
n=45;

Ak=[1,1;-0.9,-0.5];
plot(P1)
c=cell(n,1);
c{1}=P1;
for i=1:(n-1)
    P=Ak*c{i}+W;
    c{i+1}=P;
    P(1)=c{i};
    P(2)=c{i+1};
    U=Union(P);
    if P(1)>P(2)
        disp(i)
    end
    hold on
    plot(c{i+1})
end
hold on 
plot(P(1))
hold on 
