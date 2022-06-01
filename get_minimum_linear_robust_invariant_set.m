function tube=get_minimum_linear_robust_invariant_set(A,W,tresh)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
s=0;
while true
    s=s+1;
    alpha = get_alpha(A,W,s);
    if ((alpha>1)||(alpha==0))
        continue
    end
    Ms= get_MS(A,W,s);
    if alpha<(tresh/(tresh+Ms))
        break
    end
end
Fs =W;
for i=1:s
    Fs=Fs+A^i*W;
end
tube = (1-alpha)^(-1)*Fs;

end