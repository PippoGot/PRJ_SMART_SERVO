%% Initialization

close all
clc

Full_Model_params;
% Add PI.m file

%% Filter and Derivative

%Low-pass filter current Hi
H.wc = 2*pi*20;
H.delta = 1/sqrt(2);
Hi.num = H.wc^2;
Hi.den = [1 2*H.delta*H.wc H.wc^2];

%High-pass filter velocity and acceleration Hw
Hw.num = [H.wc^2 0];
Hw.den = [1 2*H.delta*H.wc H.wc^2];

%% Estimation flux constant Kphi

% Data collected from sensors: V (Motor Voltage), I (Motor Current), 
% wm (Motor Angolar speed)

% Through PI velocity control (simulink) impose a steircase velocity reference 
% to obtain the mean of the motor Kphi (at steady-state, excluding tranisent)

t = 0.01;       % Time step of the simulation [s] (Probably to cange)
t_tran1 = 0.5;  % Time of transient smaples to delate in experiment 1
t_delta1 = 2;   % Period of the step in experiment 1
M = 8;          % Number of step of the steircase function reference (To use on simulink) (maybe not necessary)

for n = 1:M
    steady.I(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1) = out.I.signals.values((t_tran1+(n-1)*t_delta1)/t:(t_delta1-t_tran1+(n-1)*t_delta1)/t,1);    % Discard  transient, leaving only steady state samples
    steady.V(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1) = out.V.signals.values((t_tran1+(n-1)*t_delta1)/t:(t_delta1-t_tran1+(n-1)*t_delta1)/t,1);
    steady.wm(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1) = out.wm.signals.values((t_tran1+(n-1)*t_delta1)/t:(t_delta1-t_tran1+(n-1)*t_delta1)/t,1);
end

for n = 1:M
    mean.I(n,1) = mean(I.signals.values(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1));     % Vector of mean value for every step
    mean.V = mean(V.signals.values(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1));
    mean.wm = mean(wm.signals.values(1+(n-1)*(t_delta1-2*t_tran1)/t:n*(t_delta1-2*t_tran1)/t,1));
end

mean.E = mean.V - motor.Ra * mean.I;
est.Kphi = mean(mean.E ./ mean.wm);

% Second version without going trough the mean
% steady.E = steady.V - motor.Ra * steady.I;
% est.motor.Kphi = mean(steady.E ./ steady.wm);
%% Estimation viscous friction coefficient B and Coulomb (static) friction component τs (tau_s) through LeastSquare method (LS)

% Through the data already collect from the previous experiment
LS.Y = est.Kphi * mean.I;       % Vector of the steady-state motor torque
LS.phi = [mean.wm 1/motor.gearbox*sign(mean.wm)];   % Forse bisogna considerare l'attrito statico del gearbox: 1/N *sign(steady.wm) (come in control lab)
LS.theta = LS.phi \ LS.Y;

est.B = Ls.theta(1,1);
est.tau_s = Ls.theta(2,1);

%% Estimation motor inertia J - Control Lab mehtod
% Notice that J is the motor inertia if we can take the mesurament directly
% on the DC motor. If the sensors need the use of the gearbox, J is going
% to be the total inertia considering the load and the gear ratio

% Through PI velocity control (simulink) impose a steircase acceleration
% refernce (makes the derivative in order to obtain the speed refernce) 

t_tran2 = 0.1;  % Time of transient smaples to delate in experiment 2
t_delta2 = 1;    % Period of the step in experiment 2

dyn.I(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1) = out.I.signals.values((t_tran2+(n-1)*t_delta2)/t:(t_delta2-t_tran2+(n-1)*t_delta2)/t,1);     % Discard  transient (dyn = dynamic)
dyn.V(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1) = out.V.signals.values((t_tran2+(n-1)*t_delta2)/t:(t_delta2-t_tran2+(n-1)*t_delta2)/t,1);
dyn.wm(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1) = out.wm.signals.values((t_tran2+(n-1)*t_delta2)/t:(t_delta2-t_tran2+(n-1)*t_delta2)/t,1);
dyn.acc(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1) = out.acc.signals.values((t_tran2+(n-1)*t_delta2)/t:(t_delta2-t_tran2+(n-1)*t_delta2)/t,1);

dyn.tau_m = est.Kphi * dyn.I;
dyn.tau_f = est.B * dyn.wm + est.tau_s/motor.gearbox * sign(dyn.wm); % possibly with gear reatio (est.tau_s / N * sign(dyn.wm))
dyn.tau_i = tau.m - tau.f;

dyn.mean.acc = mean(dyn.acc(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1));      % Vector of mean value for every step
dyn.mean.tau_i = mean(dyn.I(1+(n-1)*(t_delta2-2*t_tran2)/t:n*(t_delta2-2*t_tran2)/t,1));

for n = 1:(M/2)
    dyn.J(n,1) = (dyn.mean.tau_i(2*n,1)-dyn.mean.tau_i((2*n-1),1)) / (dyn.mean.acc(2*n,1)-dyn.mean.acc((2*n-1),1));
end

est.J = mean(dyn.J);
