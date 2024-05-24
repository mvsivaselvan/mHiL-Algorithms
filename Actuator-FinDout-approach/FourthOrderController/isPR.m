function pr = isPR(bet, ka, d, alph, Kx, ...
                   omeg, zet, mvs, ...
                   a0, a1, b0, b1, c0, c1)

% Actuator parameters: bet, ka, d, alph, Kx
% VS parameters: omeg, zet, mvs
% Controller parameters: a0, a1, b0, b1, c0, c1

d0 = a0*b0*c0*mvs/(Kx*d*alph);
d1 = a1+b1+c1-2*zet*omeg-alph-bet;

if (d0<=0 || d1<=0)
    pr = false;
    return
end

NN = conv([1 0],conv([1 a1 a0],conv([1 b1 b0],[1 c1 c0])));
DD = conv([1 alph Kx*d*alph/ka],conv([1 2*zet*omeg omeg^2],[1 d1 d0]));

m1 = NN; m1(1:2:end) = 0;
n1 = NN; n1(2:2:end) = 0;
m2 = DD; m2(2:2:end) = 0;
n2 = DD; n2(1:2:end) = 0;

PRpoly__ = conv(m1,m2) - conv(n1,n2);
PRpoly__ = PRpoly__(2:2:end-2).*[1 -1 1 -1 1 -1];

rr = roots(PRpoly__);
if ~any(imag(rr)<=1e-7 & real(rr)>0)
    pr = true;
else
    pr = false;
end