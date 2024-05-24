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
ssVS = ss([0 1; -omeg^2 -2*zet*omeg],[0; 1/mvs],[1 0],0);

%% Actuator data
ka = 9300; %lb/in
d = 25000; % lb/s/V
alph = 2*pi*25; % rad/s
bet = 2*pi*0.5; % rad/s
Kx = 1;

%% Achievable VS
nfeas = 0;
ntested = 0;
a0 = logspace(-1,1,21);
a1 = logspace(-1,1,11);
N0 = length(a0);
N1 = length(a1);
feasible_sols = [];
for l = 1:N0
    a0_ = a0(l)*omeg^2;
    for m = l:N0
        b0_ = a0(m)*omeg^2;
        for n = m:N0
            c0_ = a0(n)*omeg^2;
            for p = 1:N1
                a1_ = a1(p)*alph;
                for q = p:N1
                    b1_ = a1(q)*alph;
                    for r = q:N1
                        c1_ = a1(r)*alph;
                        
                        ntested = ntested + 1;
                        if isPR(bet, ka, d, alph, Kx, ...
                                omeg, zet, mvs, ...
                                a0_, a1_, b0_, b1_, c0_, c1_)
                            nfeas = nfeas + 1;
                            feasible_sols = [feasible_sols; ...
                             a0_,a1_,b0_,b1_,c0_,c1_];
                        end
                    end
                end
            end
        end
    end
end
fprintf('ntested = %d, nfeas = %d\n', ntested, nfeas)
