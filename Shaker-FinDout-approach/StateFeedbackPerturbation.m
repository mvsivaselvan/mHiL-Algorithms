%% Description
% Create three actuator models --- nominal, and +/- 10%  perturbations of
% the nominal. If a controller to render a passive solution for all three,
% understand what the performance penalty is

%% Bushing data
Ibushing = 2000; % (lb-s^2/in)-in^2
Itower = 1000; % (lb-s^2/in)-in^2
IVS = Ibushing;
kVSrot = 1000000; % lb-in/rad

%% Equivalent VS properties
larm = 15; % moment arm of simulator (in)
kVS = kVSrot/larm^2; % lb/in
mVS = IVS/larm^2; % lb-s^2/in
w = sqrt(kVS/mVS); % rad/s
f = w/2/pi; % Hz
zet = 0.01; % let's take 2% damping

%% Actuator data
kact = 9300; %lb/in
d = 25000; % lb/s/V
alph = 2*pi*25; % rad/s
bet = 2*pi*0.5; % rad/s
mtower = Itower/larm^2;
m = mtower;

%% Actuator state space models
Ke = 0; Kp = 0;

% Nominal
Aact1 = [0 1 0 0; 0 0 1/m 0; 0 -kact -bet d; -Ke*alph 0 -Kp*alph -alph];
Bact1 = [0 0; 0 1/m; 0 0; alph 0];
Cact1 = [1 0 0 0; 0 1 0 0; 0 0 1 0];
Dact1 = 0;

% Perturbed -10percent
Aact2 = Aact1; Aact2(3,:) = 0.9*Aact1(3,:);
Bact2 = Bact1;
Cact2 = Cact1;
Dact2 = Dact1;

% Perturbed +10percent
Aact3 = Aact1; Aact3(3,:) = 1.1*Aact1(3,:);
Bact3 = Bact1;
Cact3 = Cact1;
Dact3 = Dact1;

ssAct(:,:,1) = ss(Aact1, Bact1, Cact1, Dact1);
ssAct(:,:,2) = ss(Aact2, Bact2, Cact2, Dact2);
ssAct(:,:,3) = ss(Aact3, Bact3, Cact3, Dact3);

ssAct.InputName = {'u','w'};
ssAct.OutputName = {'x','v','F'};

%% VS state space model
AVS = [0 1; -w^2 -2*zet*w];
BVS = [0; 1/mVS];
CVS = [1 0];
DVS = [];
ssVS = ss(AVS, BVS, CVS, DVS);
ssVS.InputName = 'w';
ssVS.OutputName = 'xVS';

%% Tunable controller and closed-loop system
ssController = tunableSS('Controller',2,1,2);
ssController.InputName = {'x','F'};
ssController.OutputName = 'u';

S1 = sumblk('e = xVS - x');
%T0=connect(ssVS,ssAct(:,:,1),ssController,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});
T0=connect(ssVS,ssAct,ssController,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});

%% Constraints
s = tf('s');

errorConstr = (s+2*pi*1)^3/(s+2*pi*5)^3;
tgError = TuningGoal.Gain('w','e',errorConstr);
tgError.Stabilize = true; 

controlConstr = (s+2*pi*50)/(s+2*pi*4)*0.8;
tgControl = TuningGoal.Gain('x','u',controlConstr);
tgControl.Stabilize = false;
tgControl.Openings = {'x','F'};

tgPassivity = TuningGoal.Passivity('w','v');

%% Optimization
opt = systuneOptions('RandomStart',10, 'UseParallel', true);

[Topt,fSoft,gHard,info]=...
    systune(T0,tgError,[tgControl, tgPassivity],opt);

%% Postprocess
Huwopt_ = tf(getIOTransfer(Topt,'w','u'));
Hvwopt_ = tf(getIOTransfer(Topt,'w','v'));
Hxwopt_ = tf(getIOTransfer(Topt,'w','x'));
Huxopt_ = tf(getIOTransfer(Topt,'x','u',{'x','F'}));

if 1
figure(201),
    bode(ssVS, Hxwopt_)

figure(202),
    nyquist(Hvwopt_)

figure(203),
    bode(Huxopt_, controlConstr)
end 

%% Analyze effect of controller designed on the basis of ssAct1 on ssAct2 and ssAct3
controller = ss(Topt.Blocks.Controller);
controller.InputName = {'x','F'};
controller.OutputName = 'u';

T1=connect(ssVS,ssAct(:,:,1),controller,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});
T2=connect(ssVS,ssAct(:,:,2),controller,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});
T3=connect(ssVS,ssAct(:,:,3),controller,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});

Hxw1 = tf(getIOTransfer(T1,'w','x'));
Hxw2 = tf(getIOTransfer(T2,'w','x'));
Hxw3 = tf(getIOTransfer(T3,'w','x'));

Hvw1 = tf(getIOTransfer(T1,'w','v'));
Hvw2 = tf(getIOTransfer(T2,'w','v'));
Hvw3 = tf(getIOTransfer(T3,'w','v'));

Hew1 = tf(getIOTransfer(T1,'w','e'));
Hew2 = tf(getIOTransfer(T2,'w','e'));
Hew3 = tf(getIOTransfer(T3,'w','e'));

figure(301),
    bode(Hxw1, Hxw2, Hxw3, ssVS)

figure(302),
    nyquist(Hvw1, Hvw2, Hvw3)

figure(303),
    bode(Hew1, Hew2, Hew3)

figure(304),
    bodemag(Hew1/errorConstr, Hew2/errorConstr, Hew3/errorConstr)

% figure(305),
%     viewGoal(tgError, T2)
%% Understand evalGoal

[Hspec2,f2] = evalGoal(tgError, T2);
figure(401),
    bodemag(Hspec2, Hew2/errorConstr)
fprintf('Normalized soft constraint value for T2: %g\n',f2)