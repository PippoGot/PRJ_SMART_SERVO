%% Initialization

close all
clc

Full_Model_params;

s = tf('s');


%% Specifications

Ri.wb = 500;                    % current bandwidth                         [rad/s]
Ri.mphi = 80 * pi/180;          % current phase margin                      [rad]

Rw.tr = 0.01;                   % speed rise time                           [s]
Rw.wb = 2.2 / Rw.tr;            % speed bandwith                            [rad/s]
Rw.mphi = 60 * pi/180;          % speed phase margin                        [rad]

Rt.tr = 0.02;                   % position rise time                        [s]
Rt.wb = 2.2 / Rt.tr;            % position bandwith                         [rad/s]


%% PI Tune of Current Controller

Ai = bridge.gain * motor.Tm1 / motor.Ra;

Li1 = Ai / ((1+s*bridge.Tdelay)*(1+s*motor.Te)*(1+s*motor.Tm1));
[mag_i1, phase_i1] = bode(Li1, Ri.wb);

Ri.Tpi = tan(Ri.mphi - 180 - phase_i1) / Ri.wb;

Li2 = (1+s*Ri.Tpi) * Li1;
[mag_i2, phase_i2] = bode(Li2, Ri.wb);

Ri.Ki = 1 / mag_i1;
Ri.Kp = Ri.Ki * Ri.Tpi;

Li = Ri.Ki * Li2;
Gi = Li / (1 + Li);

figure(1);
bode(Li, Gi);


%% PI Tune of Speed Controller

Aw = motor.Kphi * motor.gearbox / motor.J;

Tw = 1e-5;
csi = 1/sqrt(2);
wlp = 500;
Sw = (s / (1 + s*Tw)) * (wlp^2 / (s^2 + 2*csi*wlp*s + wlp^2));



Lw1 = Aw * Gi / s^2;
[mag_w1, phase_w1] = bode(Lw1, Rw.wb);

Rw.Tpi = tan(Rw.mphi - 180 - phase_w1) / Rw.wb;
% Rw.Tpi = 1/350;

Lw2 = (1+s*Rw.Tpi) * Lw1;
[mag_w2, phase_w2] = bode(Lw2, Rw.wb);

Rw.Ki = 1 / mag_w1;
Rw.Kp = Rw.Ki * Rw.Tpi;

Lw = Rw.Ki * Lw2;
Gw = Lw / (1 + Lw);
% 
% figure(2)
% bode(Lw, Gw)
% margin(Lw)
% figure(3)
% margin(Gw)

Rw.Kp = 0.785;
Rw.Ki = 100;


Rw