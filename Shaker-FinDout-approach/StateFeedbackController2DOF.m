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
ktopple = -30; % lb/in
mtower = Itower/larm^2;
m = mtower;

%% Expand to 2 DOF
zero_ = zeros(2);
one_ = eye(2);

% Actuator
kact_ = kact*one_;
d_ = d*one_;
alph_ = alph*one_;
bet_ = bet*one_;
ktopple_ = ktopple*one_;
m_ = m*one_; % may eventually need to account for coupling here

%VS
w_ = w*one_;
zet_ = zet*one_;
%mVS_ = mVS*one_; % Goal is to account for coupling here
mVS_ = mVS*[0.9 0.5;0.5 1.1];
%% State space models
Aact = [zero_ one_ zero_ zero_; ...
       -m_\ktopple_ zero_ m_\one_ zero_; ...
       zero_ -kact_ -bet_ d_; ...
       zero_ zero_ zero_ -alph_];
Bact = [zero_ zero_; zero_ m_\one_; zero_ zero_; alph_ zero_];
Cact = [one_ zero_ zero_ zero_; ...
        zero_ one_ zero_ zero_; ...
        zero_ zero_ one_ zero_];
Dact = 0;
ssAct = ss(Aact, Bact, Cact, Dact);
ssAct.InputName = {'u(1)','u(2)','w(1)','w(2)'};
ssAct.OutputName = {'x(1)','x(2)','v(1)','v(2)','F(1)','F(2)'};

AVS = [zero_ one_; -w_^2 -2*zet_*w_];
BVS = [zero_; mVS_\one_];
CVS = [one_ zero_];
DVS = [];
ssVS = ss(AVS, BVS, CVS, DVS);
ssVS.InputName = {'w(1)','w(2)'};
ssVS.OutputName = {'xVS(1)','xVS(2)'};

%% Controller and constraints
% ssController = tunableSS('Controller',4,2,4,'full'); % THIS DOES NOT RESULT IN GOOD STARTING POINTS
% Use block diagonal ss model consisting of 2 1DOF solutions as the staring
% point
load controller1DOF.mat
controller0 = blkdiag(controller1DOF,controller1DOF);
ssController = tunableSS('Controller',controller0,'full');
ssController.InputName = {'x(1)','F(1)','x(2)','F(2)'};
ssController.OutputName = {'u(1)','u(2)'};

S1_1 = sumblk('e(1) = xVS(1) - x(1)');
S1_2 = sumblk('e(2) = xVS(2) - x(2)');
T0=connect(ssVS,ssAct,ssController,S1_1,S1_2,...
           {'w(1)','w(2)'},...
           {'e(1)','e(2)','x(1)','x(2)','v(1)','v(2)','F(1)','F(2)',...
            'xVS(1)','xVS(2)','u(1)','u(2)'},...
           {'x(1)','x(2)','F(1)','F(2)'});

s = tf('s');

errorConstr = (s+2*pi*1)/(s+2*pi*5);
tgError = TuningGoal.Gain({'w(1)','w(2)'},{'e(1)','e(2)'},errorConstr);
tgError.Stabilize = true; 

controlConstr = (s+2*pi*50)/(s+2*pi*4)*0.7;
tgControl = TuningGoal.Gain({'x(1)','x(2)'},{'u(1)','u(2)'},controlConstr);
tgControl.Stabilize = false;
tgControl.Openings = {'x(1)','x(2)','F(1)','F(2)'};

tgPassivity = TuningGoal.Passivity({'w(1)','w(2)'},{'v(1)','v(2)'});

opt = systuneOptions('RandomStart',10, 'UseParallel', false);

%% Evaluate 1 DOF solution
if 0
load controller1DOF.mat
controller = blkdiag(controller1DOF,controller1DOF);
controller.InputName = {'x(1)','F(1)','x(2)','F(2)'};
controller.OutputName = {'u(1)','u(2)'};

T1DOF = connect(ssVS,ssAct,controller,S1_1,S1_2,...
                {'w(1)','w(2)'},...
                {'e(1)','e(2)','x(1)','x(2)','v(1)','v(2)','F(1)','F(2)',...
                 'xVS(1)','xVS(2)','u(1)','u(2)'},...
                {'x(1)','x(2)','F(1)','F(2)'});
figure(501),
    viewGoal(tgError, T1DOF);

figure(502),
    viewGoal(tgControl, T1DOF);

figure(503),
    viewGoal(tgPassivity, T1DOF);

return
end
%% Run optimization
[Topt,fSoft,gHard,info]=...
    systune(T0,tgError,[tgControl, tgPassivity],opt);
Huwopt_ = tf(getIOTransfer(Topt,{'w(1)','w(2)'},{'u(1)','u(2)'}));
% Hvwopt_ = tf(getIOTransfer(Topt,'w','v'));
Hxwopt_ = tf(getIOTransfer(Topt,{'w(1)','w(2)'},{'x(1)','x(2)'}));
% Huxopt_ = tf(getIOTransfer(Topt,'x','u',{'x','F'}));

figure(201),
    bode(ssVS, Hxwopt_)
return
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
