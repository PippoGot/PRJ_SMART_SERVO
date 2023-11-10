%% Initialization

close all
clc

Full_Model_params;

s = tf('s');


%% PI Tune of Current Controller

Ki1 = (motor.Tm1^2 + bridge.Tdelay^2) / (2*motor.Tm1*bridge.Tdelay) - 1;

Ri.Tpi = motor.Te;
Ri.Ki = Ki1 * motor.Ra / (bridge.gain * motor.Tm1);
Ri.Kp = Ri.Tpi * Ri.Ki;

Ai = Ri.Ki * bridge.gain * motor.Tm1 / motor.Ra;
Li = Ai / ((1 + s * motor.Tm1) * (1 + s * bridge.Tdelay));
Gi = Li / (1 + Li);


%% PI Tune of Speed Controller

Rw.tr = 0.01;
Rw.wb = 2.2 / Rw.tr;
Rw.mphi = 65;

Rw.Tpi = -tan(Rw.mphi) / Rw.wb;


Lw1 = Gi * motor.Kphi * (1 + s * Rw.Tpi) / (s^2 * motor.J);
[magnitude, phase] = bode(Lw1, Rw.wb);

Rw.Ki = 1 / magnitude;
Rw.Kp = Rw.Tpi * Rw.Ki;

Lw = Rw.Ki * Lw1;
Gw = Lw / (1 + Lw);


% figure(1)
% bode(Lw, Gw)


%% P Tune of Position Controller

% ???



