%% Estimation flux constant Kphi

% Data collected from sensors: V (Motor Voltage), I (Motor Current), 
% wm (Motor Angolar speed)

% Through PI velocity control (simulink) impose a steircase velocity reference 
% to obtain the mean of the motor Kphi (at steady-state, excluding tranisent)

t = 1e-3;                    % Time step of all the simulations [s]

ex1.t_tran = 1;              % Time of transient smaples to delate in experiment 1
ex1.t_delta = 10;            % Period of the step in experiment 1
ex1.M = 20;                  % Number of step of the steircase function reference (To use on simulink) (maybe not necessary)

load("data_ex1.mat")    % Load data of experiment 1

ex1.thetam = out.data.signals(1).values(:,2);                     % [deg]
ex1.V = out.data.signals(4).values(:,2);                          % [V]
ex1.I = out.data.signals(3).values(:,2);                          % [A]
ex1.wl = out.data.signals(2).values(:,5) * conv.rpm__to__rad_s;   % from [rpm] to [rad/s]
ex1.wm = ex1.wl / motor.gearbox;                                  % [rad/s]

ex1.time = out.data.time;           % [s]

% Discard  transient, leaving only steady state samples
for n = 1:ex1.M
    steady.I(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.I((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);    
    steady.V(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.V((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);
    steady.wm(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1) = ex1.wm((ex1.t_tran+(n-1)*ex1.t_delta)/t+1:(ex1.t_delta-ex1.t_tran+(n-1)*ex1.t_delta)/t,1);
end

% Vector of mean value for every step
for n = 1:ex1.M
    average.I(n,1) = mean(steady.I(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));     
    average.V(n,1) = mean(steady.V(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));
    average.wm(n,1) = mean(steady.wm(1+(n-1)*(ex1.t_delta-2*ex1.t_tran)/t:n*(ex1.t_delta-2*ex1.t_tran)/t,1));
end

average.E = average.V - motor.Ra * average.I;
est.Kphi = mean(average.E ./ average.wm);

% Second version without going trough the mean
steady.E = steady.V - motor.Ra * steady.I;
est.motor.Kphi = mean(steady.E ./ steady.wm);

%% Plots Experiment 1

figure(1)
sgtitle('Experiment 1')
subplot(4,1,1);
plot(ex1.time,ex1.thetam)
title('Position theta_m [deg]')
grid
subplot(4,1,2);
plot(ex1.time,ex1.V)
title('Voltage [V]')
grid
subplot(4,1,3);
plot(ex1.time,ex1.I)
title('Current [I]')
grid
subplot(4,1,4);
plot(ex1.time,ex1.wm, ex1.time,ex1.wl)
title('Velocity w_l & w_m [rad/s]')
grid

%% Estimation viscous friction coefficient B and Coulomb (static) friction component τs (tau_s) through LeastSquare method (LS)
% Through the data already collect from the previous experiment

LS.Y = est.Kphi * average.I;                % Vector of the steady-state motor torque
LS.phi = [average.wm sign(average.wm)];
LS.theta = LS.phi \ LS.Y;

est.B = LS.theta(1,1);
est.tau_s = LS.theta(2,1);

%% Estimation motor inertia J - Control Lab mehtod
% To notice that the J found is the J equivalent which coniders both 
% inertia of the motor and the gear box

% Through PI velocity control (simulink) impose a steircase acceleration
% refernce (makes the derivative in order to obtain the speed refernce) 

ex2.t_tran = 0.2;           % Time of transient smaples to delate in experiment 2
ex2.t_delta = 2.5;          % Period of the step in experiment 2
ex2.M = 46;                 % Number of step of the steircase acceleration function reference

load("data_ex2.mat")    % Load data of experiment 2

ex2.thetam = out.data.signals(1).values(:,2);                     % [deg]
ex2.V = out.data.signals(4).values(:,2);                          % [V]
ex2.I = out.data.signals(3).values(:,2);                          % [A]
ex2.wl = out.data.signals(2).values(:,5) * conv.rpm__to__rad_s;   % from [rpm] to [rad/s]
ex2.wm = ex2.wl / motor.gearbox;                                  % [rad/s]
ex2.al = out.data.signals(5).values;                              % [rad/s^2]
ex2.am = ex2.al / motor.gearbox;                                  % [rad/s^2]

ex2.time = out.data.time;                                         % [s]

% Creation of indices for the selection of the window without trasient
index = ex2.t_delta/t;
i = index/2+1:index:ex2.M*index;
i3 = i-((index/2+1)-ex2.t_tran/t);
i4 = i+((index/2+1)-ex2.t_tran/t);

delta_index = index-2*(ex2.t_tran/t);
i1 = 1:delta_index:ex2.M*delta_index;
i2 = (delta_index+2:delta_index:(ex2.M+1)*delta_index)+1;

for n = 1:ex2.M
    dyn.I(i1(n):i2(n),1) = ex2.I(i3(n):i4(n),1);
    dyn.V(i1(n):i2(n),1) = ex2.V(i3(n):i4(n),1);
    dyn.wm(i1(n):i2(n),1) = ex2.wm(i3(n):i4(n),1);
    dyn.am(i1(n):i2(n),1) = ex2.am(i3(n):i4(n),1);
    dyn.time(i1(n):i2(n),1) = ex2.time(i3(n):i4(n),1);
end

dyn.tau_m = est.Kphi * dyn.I;
dyn.tau_f = est.B * dyn.wm + est.tau_s * sign(dyn.wm);
dyn.tau_i = dyn.tau_m - dyn.tau_f;

% Vector of mean value for each step
for n = 1:ex2.M
    dyn.average.am(n,1) = mean(dyn.am(uint32(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint32(n*(ex2.t_delta-2*ex2.t_tran)/t),1));
    dyn.average.tau_i(n,1) = mean(dyn.tau_i(uint32(1+(n-1)*(ex2.t_delta-2*ex2.t_tran)/t):uint32(n*(ex2.t_delta-2*ex2.t_tran)/t),1));
end

for n = 1:(ex2.M/2)
    dyn.J(n,1) = (dyn.average.tau_i(2*n-1,1)-dyn.average.tau_i(2*n,1)) / (dyn.average.am(2*n-1,1)-dyn.average.am(2*n,1));
end

est.J = mean(dyn.J);

%% Plots Experiment 2

figure(2)
sgtitle('Experiment 2')
subplot(4,1,1);
plot(ex2.time,ex2.thetam)
title('Position theta_m [deg]')
grid
subplot(4,1,2);
plot(ex2.time,ex2.V)
title('Voltage [V]')
grid
subplot(4,1,3);
plot(ex2.time,ex2.I)
title('Current [I]')
grid
subplot(4,1,4);
plot(ex2.time,ex2.wm, ex2.time,ex2.wl)
title('Velocity w_l & w_m [rad/s]')
grid

figure(3)
sgtitle('Experiment 2')
subplot(3,1,1);
plot(ex2.time,ex2.thetam)
title('Position theta_m [deg]')
grid
subplot(3,1,2);
plot(ex2.time,ex2.wm)
title('Velocity w_m [rad/s]')
grid
subplot(3,1,3);
plot(ex2.time,ex2.am)
title('Acceleration a_m [rad/s^2]')
grid

for n = 1:ex2.M
    dyn.am_mean(i1(n):i2(n),1) = dyn.average.am(n,1);
end

figure(4)
plot(dyn.time,dyn.am, dyn.time,dyn.am_mean)
title('Experiment 2 Acceleration a_m & mean(a_m) [rad/s^2]')
grid