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
zet = 0.02; % let's take 2% damping

%% Actuator data
kact = 9300; % ureal('kact',9300,'percentage',5); %lb/in
d = 25000; % ureal('d',25000,'percentage',5); % lb/s/V
alph = 2*pi*25; % ureal('alph',2*pi*25,'plusminus',[-1 5]); % rad/s
bet = 2*pi*0.5; % ureal('bet',2*pi*0.5,'percentage',5); % rad/s
mtower = Itower/larm^2;
m = mtower;

%% Optimization approach
Aact = [0 1 0 0; 0 0 1/m 0; 0 -kact -bet d; 0 0 0 -alph];
Bact = [0 0; 0 1/m; 0 0; alph 0];
Cact = [1 0 0 0; 0 1 0 0; 0 0 1 0];
Dact = 0;
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

errorConstr = (s+2*pi*1)/(s+2*pi*5);
tgError = TuningGoal.Gain('w','e',errorConstr);
tgError.Stabilize = true; 

controlConstr = (s+2*pi*50)/(s+2*pi*4)*0.7;
tgControl = TuningGoal.Gain('x','u',controlConstr);
tgControl.Stabilize = false;
tgControl.Openings = {'x','F'};

tgPassivity = TuningGoal.Passivity('w','v');

opt = systuneOptions('RandomStart',10, 'UseParallel', false);

[Topt,fSoft,gHard,info]=...
    systune(T0,tgError,[tgControl, tgPassivity],opt);

controller = ss(Topt.blocks.Controller);

%% Controller for nonlinear VS
A0 = [controller.A zeros(2); zeros(2) [0 1; -w^2 -2*zet*w]];
B0 = [controller.B zeros(2,1); zeros(1,3); zeros(1,2) 1/mVS];
C0 = [controller.C zeros(1,2); 0 0 1 0; 0 0 0 1];
D0 = [controller.D 0; 0 0 0; 0 0 0];
ssControllerNVS = tunableSS('Controller',ss(A0,B0,C0,D0),'full');
ssControllerNVS.A.Free(3,:) = false;
ssControllerNVS.B.Free(3,:) = false;
ssControllerNVS.C.Free(2:3,:) = false;
ssControllerNVS.D.Free(2:3,:) = false;
ssControllerNVS.InputName = {'x','F','w2'};
ssControllerNVS.OutputName = {'u','x2','v2'};

AVSexpanded = [0 1; -w^2 -2*zet*w];
BVSexpanded = [0 0; 1/mVS 1/mVS];
CVSexpanded = [1 0; 1 0];
DVSexpanded = [];
ssVSexpanded = ss(AVSexpanded, BVSexpanded, CVSexpanded, DVSexpanded);
ssVSexpanded.InputName = {'w', 'w2'};
ssVSexpanded.OutputName = {'xVS','xVS2'};

S2 = sumblk('e2 = xVS2 - x2');
TNVS0 = connect(ssAct,ssVSexpanded,ssControllerNVS,S1,S2,{'w','w2'},...
                             {'e','e2','x','x2','v','v2','u'},{'x','F'});

tgErrorNVS = TuningGoal.Gain({'w','w2'},{'e','e2'},errorConstr);
tgErrorNVS.Stabilize = true; 

tgControlNVS = TuningGoal.Gain('x','u',controlConstr);
tgControlNVS.Stabilize = false;
tgControlNVS.Openings = {'x','F'};

tgPassivityNVS = TuningGoal.Passivity({'w','w2'},{'v','v2'});

fprintf('Optimizing for nonlinear VS ...\n')
[TNVSopt,fSoftNVS,gHardNVS,infoNVS]=...
    systune(TNVS0,tgErrorNVS,[tgControlNVS, tgPassivityNVS]);

Hxwopt_ = tf(getIOTransfer(TNVSopt,{'w','w2'},{'x','x2'}));

controllerNVS = ss(TNVSopt.Blocks.Controller);

figure(501),
    bode(ssVSexpanded,Hxwopt_)

return
%%
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

figure(305),
    bode(controller(2))

%% Digital controller
controllerd = c2d(controller, 0.001, 'tustin');
return
addpath '..\matlab2VeriStand\'
writeController2ParamFile(controllerd, '.\ControllerParm.txt');
rmpath '..\matlab2VeriStand\'
