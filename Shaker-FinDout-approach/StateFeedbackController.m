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
Kp = 0.005;
Ke = 1;
mtower = Itower/larm^2;
m = mtower;

%% Optimization approach
Ke = 0; Kp = 0;
Aact = [0 1 0 0; 0 0 1/m 0; 0 -kact -bet d; -Ke*alph 0 -Kp*alph -alph];
Bact = [0 0; 0 1/m; 0 0; alph 0];
Cact = [1 0 0 0; 0 1 0 0; 0 0 1 0];
Dact = [];
ssAct = ss(Aact, Bact, Cact, Dact);
ssAct.InputName = {'u','w'};
ssAct.OutputName = {'x','v','F'};

AVS = [0 1; -w^2 -2*zet*w];
BVS = [0; 1/mVS];
CVS = [1 0];
DVS = [];
ssVS = ss(AVS, BVS, CVS, DVS);
ssVS.InputName = 'w';
ssVS.OutputName = 'xVS';

ssController = tunableSS('Controller',2,1,2);
ssController.InputName = {'x','F'};
ssController.OutputName = 'u';

S1 = sumblk('e = xVS - x');
T0=connect(ssVS,ssAct,ssController,S1,'w',{'e','x','v','F','xVS','u'},{'x','F'});

s = tf('s');

errorConstr = (s+2*pi*30)^3/(s+2*pi*1000)^3;
tgError = TuningGoal.Gain('w','e',errorConstr);
tgError.Stabilize = true; 

controlConstr = (s+2*pi*50)/(s+2*pi*4)*0.8;
tgControl = TuningGoal.Gain('x','u',controlConstr);
tgControl.Stabilize = false;
tgControl.Openings = {'x','F'};

tgPassivity = TuningGoal.Passivity('w','v');

opt = systuneOptions('RandomStart',20);

[Topt,fSoft,gHard,info]=...
    systune(T0,tgError,[tgControl, tgPassivity],opt);
Huwopt_ = tf(getIOTransfer(Topt,'w','u'));
Hvwopt_ = tf(getIOTransfer(Topt,'w','v'));
Hxwopt_ = tf(getIOTransfer(Topt,'w','x'));
Huxopt_ = tf(getIOTransfer(Topt,'x','u',{'x','F'}));

figure(201),
    bode(ssVS, Hxwopt_)

figure(202),
    nyquist(Hvwopt_)

figure(203),
    bode(Huwopt_, controlConstr)

%% Passivity analysis of hydraulic system
controller = ss(Topt.blocks.Controller);
controller.InputName = {'x','F'};
controller.OutputName = 'uv';

% Rewrite hydraulic system dynamics with the coordinate transformation, 
% z = F + ka*x, so that
% \dot{z} = -beta*z + d*xv + beta*ka*x
% \dot{x}_v = -alpha*xv + alpha*uv
% Output, F = z - ka*x

AHyd = [-bet d; 0 -alph];
BHyd = [bet*kact 0; 0 alph];
CHyd = [1 0];
DHyd = [-kact 0];
ssHyd = ss(AHyd, BHyd, CHyd, DHyd);
ssHyd.InputName = {'x','uv'};
ssHyd.OutputName = 'F';

ssHydCL = connect(ssHyd, controller, 'x', 'F');

figure(301),
    bode(ssHydCL)

s = tf('s');
figure(302),
    nyquist(-ssHydCL/s)

figure(303),
    bode(-ssHydCL/s)

figure(304),
    bode(controller(1), controlConstr)

%% Robustness analysis of passivity (positive realness) of control solution
betu = ureal('beta',bet,'percentage',10);
kactu = ureal('kact',kact,'percentage',10);
du = ureal('d',d,'percentage',10);
alphu = ureal('alph',alph,'plusminus',[-1 5]);
taud = 0.001; % sample delay

AHydu = [-betu du; 0 -alphu];
BHydu = [betu*kactu 0; 0 alphu];
CHydu = [1 0];
DHydu = [-kactu 0];
ssHydu = ss(AHydu, BHydu, CHydu, DHydu);
ssHydu.InputName = {'x','uv'};
ssHydu.OutputName = 'F';

% introduce sample delay in the controller and do a pade approximation of
% the delay
controllerd = controller;
controllerd.OutputDelay = taud;
%controllerd = pade(controllerd, 2);

ssHydCLu = connect(ssHydu, controllerd, 'x', 'F');

[ssHydCLu_samples, sampleValues] = usample(ssHydCLu,10);

figure(401),
    nyquist(-ssHydCLu_samples/s)
