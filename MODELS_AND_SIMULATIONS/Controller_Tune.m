%% s Variable Definition

s = tf('s');


%% PI Tune of Current Controller

%vvv 
Ai = bridge.gain * motor.Tm1 / motor.Ra;

Li1 = Ai / ((1+s*bridge.Tdelay)*(1+s*motor.Te)*(1+s*motor.Tm1));
[mag_i1, phase_i1] = bode(Li1, Ri.wb);

Ri.Tpi = tan(Ri.mphi - 180 - phase_i1) / Ri.wb;

Li2 = (1+s*Ri.Tpi) * Li1;
[mag_i2, phase_i2] = bode(Li2, Ri.wb);

Ri.Ki = 1 / mag_i2;
Ri.Kp = Ri.Ki * Ri.Tpi;

Li = Ri.Ki * Li2;
Gi = Li / (1 + Li);
%^^^

% Ai = motor.Tm1 * bridge.gain / motor.Ra;
% 
% Li1 = Ai * s / ((1+s*bridge.Tdelay)*(1+s*motor.Te)*(1+s*motor.Tm1));
% 
% Ri.Tpi = motor.Tm1;
% 
% K1 = (motor.Tm1^2 + bridge.Tdelay^2) / (2*motor.Tm1*bridge.Tdelay) - 1;
% 
% Ri.Ki = K1 / Ai;
% Ri.Kp = Ri.Ki * Ri.Tpi;
% 
% Li = Ri.Ki * (1 + s*Ri.Tpi) * Li1 / s;
% Gi = Li / (1 + Li);

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


%% P Tune of Position Controller

At = 1;

Lt1 = At * Gw / s;
[mag_t1, phase_t1] = bode(Lt1, Rt.wb);

Rt.Kp = 1/mag_t1;

Lt = Rt.Kp * Lt1;
Gt = Lt / (1 + Lt);

