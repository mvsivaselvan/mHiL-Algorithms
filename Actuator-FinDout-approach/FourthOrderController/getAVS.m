function [ssAVS, tfAVS, d0, d1] = getAVS(bet, ka, d, alph, Kx, ...
                                 omeg, zet, mvs, ...
                                 a0, a1, b0, b1, c0, c1)

% Actuator parameters: bet, ka, d, alph, Kx
% VS parameters: omeg, zet, mvs
% Controller parameters: a0, a1, b0, b1, c0, c1

d0 = a0*b0*c0*mvs/(Kx*d*alph);
d1 = a1+b1+c1-2*zet*omeg-alph-bet;

NN = conv([1 a1 a0],conv([1 b1 b0],[1 c1 c0]));
DD = conv([ka ka*alph Kx*d*alph],conv([1 2*zet*omeg omeg^2],[1 d1 d0]));

tfAVS = tf(NN,DD);
ssAVS = ss(tfAVS);
