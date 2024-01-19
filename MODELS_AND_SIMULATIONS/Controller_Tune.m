%% s Variable Definition

s = tf('s');

%% Transfer Function

PE = bridge.gain / (1 + s*bridge.Tdelay);   % H Bridge TF
M1 = (motor.B + s*motor.J) / ((motor.B + s*motor.J)*(motor.Ra + s*motor.La) + motor.Kphi^2);
M2 = motor.Kphi * motor.gearbox / (motor.B + s*motor.J);


%% PI Tune of Current Controller

Li1 = PE * M1 / s;
[mag1_i, phase1_i] = bode(Li1, Ri.wb);

Ri.Tpi = tan(Ri.mphi - 180 + phase1_i) / Ri.wb;

Li2 = (1+s*Ri.Tpi) * Li1;
[mag2_i, phase2_i] = bode(Li2, Ri.wb);

Ri.Ki = 1 / mag2_i;
Ri.Kp = Ri. Ki * Ri.Tpi;

Li = Ri.Ki * Li2;
Gi = Li / (1 + Li);


%% PI Tune of Speed Controller

Lw1 = Gi * M2 / s;
[mag1_w, phase1_w] = bode(Lw1, Rw.wb);

Rw.Tpi = tan(Rw.mphi - 180 + phase1_w) / Rw.wb;

Lw2 = (1+s*Rw.Tpi) * Lw1;
[mag2_w, phase2_w] = bode(Lw2, Rw.wb);

Rw.Ki = 1 / mag2_w;
Rw.Kp = Rw.Ki * Rw.Tpi;

Lw = Rw.Ki * Lw2;
Gw = Lw / (1 + Lw);

%% P Tune of Position Controller

% Lt1 = Gw / s;
% [mag_t1, phase_t1] = bode(Lt1, Rt.wb);
% 
% Rt.Kp = 1/mag_t1;
% 
% Lt = Rt.Kp * Lt1;
% Gt = Lt / (1 + Lt);

Lt1 = Gw / s^2;
[mag1_t, phase1_t] = bode(Lt1, Rt.wb);

Rt.Tpi = tan(Rt.mphi - 180 + phase1_t) / Rt.wb;

Lt2 = (1 + s*Rt.Tpi) * Lt1;
[mag2_t, phase2_t] = bode(Lt2, Rt.wb);

Rt.Ki = 1 / mag2_t;
Rt.Kp = Rt.Ki * Rt.Tpi;

Lt = Rt.Ki * Lt2;
Gt = Lt / (1 + Lt);
