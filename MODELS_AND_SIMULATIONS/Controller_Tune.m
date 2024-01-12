%% Initialization

close all
clc

Full_Model_params;

s = tf('s');


%% Specifications

Ri.wb = 55;                     % current bandwidth                         [rad/s]
Ri.mphi = 50 * pi/180;          % current phase margin                      [rad]

Rw.wb = 20;                     % speed bandwith                            [rad/s]
Rw.mphi = 100 * pi/180;         % speed phase margin                        [rad]

Rt.wb = 10;                     % position bandwith                         [rad/s]
Rt.mphi = 70 * pi/180;


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

% figure(1);
% bode(Li, Gi);


%% PI Tune of Speed Controller

Aw = motor.Kphi * motor.gearbox / motor.J;

Lw1 = Aw * Gi / s^2;
[mag_w1, phase_w1] = bode(Lw1, Rw.wb);

Rw.Tpi = tan(Rw.mphi - 180 - phase_w1) / Rw.wb;

Lw2 = (1+s*Rw.Tpi) * Lw1;
[mag_w2, phase_w2] = bode(Lw2, Rw.wb);

Rw.Ki = 1 / mag_w1;
Rw.Kp = Rw.Ki * Rw.Tpi;

Lw = Rw.Ki * Lw2;
Gw = Lw / (1 + Lw);

% figure(2)
% bode(Lw, Gw)


%% P Tune of Position Controller

At = 1;

Lt1 = At * Gw / s;
[mag_t1, phase_t1] = bode(Lt1, Rt.wb);

Rt.Kp = 1/mag_t1;

Lt = Rt.Kp * Lt1;
Gt = Lt / (1 + Lt);

% figure(3)
% bode(Lt, Gt)

Ri, Rw, Rt
