%% Bushing data
Ibushing = 4500; % (lb-s^2/in)-in^2
Itower = 1000; % (lb-s^2/in)-in^2
IVS = Ibushing - Itower;
kVSrot = 5600000; % lb-in/rad

%% Equivalent VS properties
larm = 15; % moment arm of simulator (in)
kvs = kVSrot/larm^2; % lb/in
mvs = IVS/larm^2; % lb-s^2/in
omeg = sqrt(kvs/mvs); % rad/s
fvs = omeg/2/pi; % Hz
zet = 0.02; % let's take 2% damping

%% Actuator data
ka = 9300; %lb/in
d = 25000; % lb/s/V
alph = 2*pi*25; % rad/s
bet = 2*pi*0.5; % rad/s
Kx = 1;

%% Achievable VS
a0 = (0.1:0.1:100)*omeg^2;
a1 = (0.01:0.01:0.1)*omeg;
[a0,a1] = meshgrid(a0,a1);
C = zeros(size(a0));
for m = 1:size(a0,1)
    for n = 1:size(a0,2)
        a0_ = a0(m,n);
        a1_ = a1(m,n);
        PRpoly = getPRpoly(bet, ka, d, alph, Kx, omeg, zet, mvs, a0_, a1_);
        r = roots(PRpoly);
        if ~any(imag(r)<=1e-7 & real(r)>0) % positive real root
            C(m,n) = 1;
        else
            C(m,n) = NaN;
        end
    end
end
figure(101),
    h = pcolor(a0/omeg^2, a1/omeg, C);
    h.EdgeColor = 'none';

%% One set of parameters that works
% From the figure,
a0_ = 100*omeg^2;
a1_ = 0.08*omeg;
b0_ = Kx*d*alph/mvs/a0_;
b1_ = 2*zet*omeg+alph+bet-a1_;
NN = conv([1 a1_ a0_],[1 b1_ b0_]);
DD = ka*conv([1 alph d*Kx*alph/ka],[1 2*zet*omeg omeg^2]);
HAVS = tf(NN,DD);
HVS = tf(1,mvs*[1 2*zet*omeg omeg^2]);
figure(201),
    bode(HVS, HAVS)