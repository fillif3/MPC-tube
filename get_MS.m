function Ms= get_MS(A,W,s)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Ms=0;
dim =length(A);
for i=1:dim
    val=0;
    basis_vec = zeros(dim,1);
    basis_vec(i)=1;
    for j=1:s
        val = val+hw((A^j)'*basis_vec,W);
    end
    Ms=max(Ms,val);
    val=0;
    basis_vec = -basis_vec;
    for j=1:s
        val = val+hw((A^j)'*basis_vec,W);
    end

end

end