%% Bushing data
Ibushing = 4500; % (lb-s^2/in)-in^2
Itower = 1000; % (lb-s^2/in)-in^2
IVS = Ibushing;
kVSrot = 5600000; % lb-in/rad

%% Equivalent VS properties
larm = 15; % moment arm of simulator (in)
kVS = kVSrot/larm^2; % lb/in
mVS = IVS/larm^2; % lb-s^2/in
w = sqrt(kVS/mVS); % rad/s
f = w/2/pi; % Hz
zet = 0.02; % let's take 2% damping

%% Actuator data
kact = 9300; %lb/in
d = 25000; % lb/s/V
alph = 2*pi*25; % rad/s
bet = 2*pi*0.5; % rad/s
Kp = 0.005;
Ke = 1;
mtower = Itower/larm^2;
m = mtower;

%% Controller parameters and constraints
b1_ = linspace(0.1,2,200)*w;
b0_ = linspace(25,100,1000)*w^2;
[b1,b0] = meshgrid(b1_,b0_);
rho = m/mVS;
a0 = rho*b0;
a1 = b1+2*zet*w;

constr1 = a1.*b0 - a0.*b1 + (2*zet*w)*(w^2 - a0 - b0 + a1.*b1) > 0;
constr2 = (-(2*zet*w)*(a0.*b0) + (w^2)*(a1.*b0 - a0.*b1)) < 0;

constr = constr1 & constr2;

constr = double(constr);
constr(constr==0) = NaN;
figure(1), 
    h = pcolor(b1/w, b0/w^2, constr);
    h.EdgeColor = 'none';
    xlabel('b_1'),
    ylabel('b_0')

%% Test one choice of parameters
b1 = 0.4*w;
b0 = 50*w^2;
a0 = rho*b0;
a1 = b1+2*zet*w;

% attainable VS
N1 = [1 a1 a0];
D1 = conv([1 2*zet*w w^2],[1 b1 b0]);
Hattained = (1/m)*tf(N1,D1);
HVS = (1/mVS)*tf(1,[1 2*zet*w w^2]);
figure(101),
    bode(HVS, Hattained)
Yattained = tf([N1 0],D1); % admittance
figure(102),
    nyquist(Yattained)

% Controller 
N2 = [1 alph+bet alph*(bet+Kp*d)];
D2 = [1 alph+bet alph*(bet+Kp*d)+kact/m (kact/m)*alph d*Ke*alph/m];
NC = conv(N1,D2)-conv(N2,D1);
NC = NC(3:end);
DC = D1;
Hcontroller = 1/(d*Ke*alph)*tf(NC,DC);
figure(103),
    bode(Hcontroller)

%% Observability and controllability of this system
% Aattained = [Aact Bact(:,1)*ssController.C; zeros(size(ssController.A,1),size(ssAct.A,2)) ssController.A];
% Battained = [Bact(:,2)+Bact(:,1)*ssController.D; ssController.B];
% Cattained = eye(8,1)';
% Dattained = [];
% ssAttained = ss(Aattained, Battained, Cattained, Dattained);
% figure, bode(ssAttained, Hattained)
% Hattained
% tf(minreal(ssAttained))
% [V__,D__] = eig(Aattained);
% [abs((Cattained*V__).' ) abs(V__\Battained)]
% ((Cattained*V__).').*(V__\Battained)

%% Optimization approach
Aact = [0 1 0 0; 0 0 1/m 0; 0 -kact -bet d; -Ke*alph 0 -Kp*alph -alph];
Bact = [0 0; 0 1/m; 0 0; Ke*alph 0];
Cact = [1 0 0 0; 0 1 0 0];
Dact = [];
ssAct = ss(Aact, Bact, Cact, Dact);
ssAct.InputName = {'u','w'};
ssAct.OutputName = {'x','v'};

AVS = [0 1; -w^2 -2*zet*w];
BVS = [0; 1/mVS];
CVS = [1 0];
DVS = [];
ssVS = ss(AVS, BVS, CVS, DVS);
ssVS.InputName = 'w';
ssVS.OutputName = 'xVS';

ssController = tunableSS('Controller',4,1,1);
ssController.InputName = 'w';
ssController.OutputName = 'u';

S1 = sumblk('e = xVS - x');
T0 = connect(ssVS, ssAct, ssController, S1, 'w', {'e','x','v','xVS','u'});

s = tf('s');

errorConstr = (s+2*pi*30)^3/(s+2*pi*1000)^3;
tgError = TuningGoal.Gain('w','e',errorConstr);
tgError.Stabilize = false; 

controlConstr = (s+2*pi*200)^3/(s+2*pi*100)^3/80;
tgControl = TuningGoal.Gain('w','u',controlConstr);
tgControl.Stabilize = false;

tgPassivity = TuningGoal.Passivity('w','v');

opt = systuneOptions('RandomStart',20);

[Topt,fSoft,gHard,info]=...
    systune(T0,tgError,[tgPassivity,tgControl],opt);
Huwopt_ = tf(getIOTransfer(Topt,'w','u'));
Hvwopt_ = tf(getIOTransfer(Topt,'w','v'));
Hxwopt_ = tf(getIOTransfer(Topt,'w','x'));

figure(201),
    bode(ssVS, Hxwopt_)

figure(202),
    nyquist(Hvwopt_)

figure(203),
    bode(Huwopt_, Hcontroller, controlConstr)