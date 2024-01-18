%% Estimation flux constant Kphi

% Data collected from sensors: V (Motor Voltage), I (Motor Current), 
% wm (Motor Angolar speed)

% Through PI velocity control (simulink) impose a steircase velocity reference 
% to obtain the mean of the motor Kphi (at steady-state, excluding tranisent)

t = 1e-3;                   % Time step of the simulation [s]

%ex1.data = readtable('../DATA/Motor Parameters Estimation Data/Log_File_Staircase.txt','DecimalSeparator',',');
%index = find(ismissing(ex1.data(:,"timestamp")));
load("staircase_data.mat")

ex1.t_tran = 1;              % Time of transient smaples to delate in experiment 1
ex1.t_delta = 10;               % Period of the step in experiment 1
ex1.M = 20;                      % Number of step of the steircase function reference (To use on simulink) (maybe not necessary)

%ex1.data=ex1.data(index(end)+1 : index(end)+ex1.t_delta*ex1.M/t ,:);

ex1.thetam = reshape(data.signals(1).values,[],1);
ex1.I_filtered = reshape(data.signals(4).values,[],1);
ex1.V_filtered = reshape(data.signals(5).values,[],1);
ex1.wl = data.signals(2).values(:,2) * conv.rpm__to__rad_s;     % from [rpm] to [rad/s]
ex1.wm = ex1.wl / motor.gearbox;                                % [rad/s]

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
plot(ex1.time,ex1.V_filtered)
grid
subplot(3,1,3);
plot(ex1.time,ex1.I_filtered)
grid

%%
%{
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
%}

%% Estimation viscous friction coefficient B and Coulomb (static) friction component τs (tau_s) through LeastSquare method (LS)

% Through the data already collect from the previous experiment
LS.Y = est.Kphi * average.I;                                % Vector of the steady-state motor torque
LS.phi = [average.wm sign(average.wm)];     % Forse bisogna considerare l'attrito statico del gearbox: motor.gearbox *sign(steady.wm) (come in control lab)
LS.theta = LS.phi \ LS.Y;

est.B = LS.theta(1,1);
est.tau_s = LS.theta(2,1);

%% Estimation motor inertia J - Control Lab mehtod
% Notice that J is the motor inertia if we can take the mesurament directly
% on the DC motor. If the sensors need the use of the gearbox, J is going
% to be the total inertia considering the load and the gear ratio

% Through PI velocity control (simulink) impose a steircase acceleration
% refernce (makes the derivative in order to obtain the speed refernce) 

%ex2.data = readtable('../DATA/Motor Parameters Estimation Data/Log_File_Triangle.txt','DecimalSeparator',',');
load("triangle_data.mat")

ex2.t_tran = 0.2;              % Time of transient smaples to delate in experiment 2
ex2.t_delta = 2.5;               % Period of the step in experiment 2
ex2.M = 46;

%ex2.data=ex2.data(ex2.t_delta*2/t+1:ex2.t_delta*8/t,:);     % Selection of desire intervals (descard first and last ramp). In case of new data it needed to change this selection

ex2.thetam = reshape(data.signals(1).values,[],1);
ex2.V = reshape(data.signals(5).values,[],1);
ex2.I = reshape(data.signals(4).values,[],1);
ex2.wl = data.signals(2).values(:,2) * conv.rpm__to__rad_s;         % from [rpm] to [rad/s]
ex2.wm = ex2.wl / motor.gearbox;                                    % [rad/s]

ex2.time = linspace(0, size(ex2.thetam,1)*t, size(ex2.thetam,1));
ex2.am = gradient(ex2.wm,ex2.time);                                 % [rad/s^2]
ex2.am_filtered = lowpass(ex2.am, 10, 1/t);

%{
for n = 1:ex2.M
    dyn.I(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.I(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);     % Discard  transient (dyn = dynamic)
    dyn.V(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.V(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
    dyn.wm(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.wm(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
    dyn.am(uint16(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint16(n*(ex2.t_delta-2*ex2.t_tran)/t),1) = ex2.am(uint16((ex2.t_tran+(n-1)*ex2.t_delta)/t+1):uint16((ex2.t_delta-ex2.t_tran+(n-1)*ex2.t_delta)/t),1);
end
%}

i = [1251:2500:ex2.M*2500];
i3 = i-(1251-200);
i4 = i+(1251-200);

i1 = [1:2100:ex2.M*2100];
i2 = [2102:2100:(ex2.M+1)*2100]+1;

for n = 1:ex2.M
    dyn.I(i1(n):i2(n),1) = ex2.I(i3(n):i4(n),1);
    dyn.V(i1(n):i2(n),1) = ex2.V(i3(n):i4(n),1);
    dyn.wm(i1(n):i2(n),1) = ex2.wm(i3(n):i4(n),1);
    dyn.am(i1(n):i2(n),1) = ex2.am(i3(n):i4(n),1);
end

dyn.tau_m = est.Kphi * dyn.I;
dyn.tau_f = est.B * dyn.wm + est.tau_s * sign(dyn.wm); % possibly with gear reatio (est.tau_s * motor.gearbox * sign(dyn.wm))
dyn.tau_i = dyn.tau_m - dyn.tau_f;

for n = 1:ex2.M
    dyn.average.am(n,1) = mean(dyn.am(uint32(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint32(n*(ex2.t_delta-2*ex2.t_tran)/t),1));      % Vector of mean value for every step
    dyn.average.tau_i(n,1) = mean(dyn.tau_i(uint32(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint32(n*(ex2.t_delta-2*ex2.t_tran)/t),1));
end

for n = 1:(ex2.M/2)
    dyn.J(n,1) = (dyn.average.tau_i(2*n-1,1)-dyn.average.tau_i(2*n,1)) / (dyn.average.am(2*n-1,1)-dyn.average.am(2*n,1));
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

%{
figure(5)
subplot(2,1,1);
plot(ex2.time,ex2.V, ex2.time,ex2.V_filtered)
grid
subplot(2,1,2); 
plot(ex2.time,ex2.I, ex2.time,ex2.I_filtered)
grid
%}

figure(6)
subplot(2,1,1);
plot(ex2.time,ex2.wm)
grid
subplot(2,1,2);
plot(ex2.time,ex2.am, ex2.time,ex2.am_filtered)
grid