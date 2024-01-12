%% Initialization

close all
clear
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

t = 1e-3;                   % Time step of the simulation [s]

ex1.data = readtable('../DATA/Motor Parameters Estimation Data/Log_File_Staircase.txt','DecimalSeparator',',');
index = find(ismissing(ex1.data(:,"timestamp")));

ex1.t_tran = 0.5;              % Time of transient smaples to delate in experiment 1
ex1.t_delta = 5;               % Period of the step in experiment 1
ex1.M = 8;                      % Number of step of the steircase function reference (To use on simulink) (maybe not necessary)

ex1.data=ex1.data(index(end)+1 : index(end)+ex1.t_delta*ex1.M/t ,:);

ex1.thetam = ex1.data.Angle_DEG;
ex1.I = ex1.data.Current_A;
ex1.V = ex1.data.Voltage_V;
ex1.V_filtered = lowpass(ex1.V, 10, 1/t);
ex1.I_filtered = lowpass(ex1.I, 10, 1/t);
ex1.wm = gradient(ex1.thetam);
ex1.wm_filtered = lowpass(ex1.wm, 10, 1/t);

ex1.elements = size(ex1.V,1);
ex1.time = linspace(0, size(ex1.thetam,1)*t, size(ex1.thetam,1));

for n = 1:ex1.M
    steady.I(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.I_filtered((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);    % Discard  transient, leaving only steady state samples
    steady.V(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.V_filtered((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);
    steady.wm(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.wm((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);
end

for n = 1:ex1.M
    average.I(n,1) = mean(steady.I(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));     % Vector of mean value for every step
    average.V(n,1) = mean(steady.V(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));
    average.wm(n,1) = mean(steady.wm(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));
end

average.E = average.V - motor.Ra * average.I;
est.Kphi = mean(average.E ./ average.wm);

% Second version without going trough the mean
% steady.E = steady.V - motor.Ra * steady.I;
% est.motor.Kphi = mean(steady.E ./ steady.wm);

%% Plots Experiment 1

figure(1)
subplot(3,1,1);
plot(ex1.time,ex1.thetam)
grid
subplot(3,1,2);
plot(ex1.time,ex1.V)
grid
subplot(3,1,3);
plot(ex1.time,ex1.I)
grid

figure(2)
subplot(2,1,1);
plot(ex1.time,ex1.V, ex1.time,ex1.V_filtered)
grid
subplot(2,1,2);
plot(ex1.time,ex1.I, ex1.time,ex1.I_filtered)
grid

figure(3)
subplot(2,1,1);
plot(ex1.time,ex1.thetam)
grid
subplot(2,1,2);
plot(ex1.time,ex1.wm, ex1.time,ex1.wm_filtered)
grid

%% Estimation viscous friction coefficient B and Coulomb (static) friction component τs (tau_s) through LeastSquare method (LS)

% Through the data already collect from the previous experiment
LS.Y = est.Kphi * average.I;                                % Vector of the steady-state motor torque
LS.phi = [average.wm 1/motor.gearbox*sign(average.wm)];     % Forse bisogna considerare l'attrito statico del gearbox: 1/N *sign(steady.wm) (come in control lab)
LS.theta = LS.phi \ LS.Y;

est.B = LS.theta(1,1);
est.tau_s = LS.theta(2,1);

%% Estimation motor inertia J - Control Lab mehtod
% Notice that J is the motor inertia if we can take the mesurament directly
% on the DC motor. If the sensors need the use of the gearbox, J is going
% to be the total inertia considering the load and the gear ratio

% Through PI velocity control (simulink) impose a steircase acceleration
% refernce (makes the derivative in order to obtain the speed refernce) 

ex2.data = readtable('../DATA/Motor Parameters Estimation Data/Log_File_Triangle.txt','DecimalSeparator',',');

ex2.t_tran = 0.2;              % Time of transient smaples to delate in experiment 2
ex2.t_delta = 2;               % Period of the step in experiment 2
ex2.M = 6;

ex2.data=ex2.data(ex2.t_delta*2/t+1:ex2.t_delta*8/t,:);     % Selection of desire intervals (descard first and last ramp). In case of new data it needed to change this selection

ex2.thetam = ex2.data.Angle_DEG;
ex2.I = ex2.data.Current_A;
ex2.V = ex2.data.Voltage_V;

ex2.V_filtered = lowpass(ex2.V, 10, 1/t);
ex2.I_filtered = lowpass(ex2.I, 10, 1/t);
ex2.wm = gradient(ex2.thetam);
ex2.wm_filtered = lowpass(ex2.wm, 10, 1/t);
ex2.am = gradient(ex2.wm_filtered);
ex2.am_filtered = lowpass(ex2.am, 10, 1/t);

ex2.elements = size(ex2.V,1);
ex2.time = linspace(0, size(ex2.thetam,1)*t, size(ex2.thetam,1));

for n = 1:ex2.M
    dyn.I(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.I(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);     % Discard  transient (dyn = dynamic)
    dyn.V(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.V(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
    dyn.wm(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.wm(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
    dyn.am(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.am(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
end

dyn.tau_m = est.Kphi * dyn.I;
dyn.tau_f = est.B * dyn.wm + est.tau_s/motor.gearbox * sign(dyn.wm); % possibly with gear reatio (est.tau_s / N * sign(dyn.wm))
dyn.tau_i = dyn.tau_m - dyn.tau_f;

for n = 1:ex2.M
    dyn.average.am(n,1) = mean(dyn.am(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),uint16(1)));      % Vector of mean value for every step
    dyn.average.tau_i(n,1) = mean(dyn.I(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),uint16(1)));
end

for n = 1:(ex2.M/2)
    dyn.J(n,1) = (dyn.average.tau_i(2*n,1)-dyn.average.tau_i((2*n-1),1)) / (dyn.average.am(2*n,1)-dyn.average.am((2*n-1),1));
end

est.J = mean(dyn.J);

%% Plots Experiment 2

figure(4)
subplot(3,1,1);
plot(ex2.time,ex2.thetam)
grid
subplot(3,1,2);
plot(ex2.time,ex2.V)
grid
subplot(3,1,3);
plot(ex2.time,ex2.I)

figure(5)
subplot(2,1,1);
plot(ex2.time,ex2.V, ex2.time,ex2.V_filtered)
grid
subplot(2,1,2); 
plot(ex2.time,ex2.I, ex2.time,ex2.I_filtered)
grid

figure(6)
subplot(2,1,1);
plot(ex2.time,ex2.wm, ex2.time,ex2.wm_filtered)
grid
subplot(2,1,2);
plot(ex2.time,ex2.am, ex2.time,ex2.am_filtered)
grid