%% s Variable Definition

s = tf('s');


%% PI Tune of Current Controller

m_pole = (motor.B+s*motor.J);
e_pole = (motor.Ra+s*motor.La);
% 
% Li = (bridge.gain / (1 + s*bridge.Tdelay)) * (m_pole / (m_pole * e_pole + motor.Kphi^2));
% Li1 = Li / s;
% [mag_i1, phase_i1] = bode(Li1, Ri.wb);
% 
% Ri.Tpi = tan(Ri.mphi - 180 - phase_i1) / Ri.wb;
% 
% Li2 = (1+s*Ri.Tpi) * Li1;
% [mag_i2, phase_i2] = bode(Li2, Ri.wb);
% 
% Ri.Ki = 1 / mag_i2;
% Ri.Kp = Ri.Ki * Ri.Tpi;
% 
% Gi = Ri.Ki * Li2;

Ai = motor.Tm1 * bridge.gain / motor.Ra;

Li1 = Ai * s / ((1+s*bridge.Tdelay)*(1+s*motor.Te)*(1+s*motor.Tm1));

Ri.Tpi = motor.Te;

K1 = (motor.Tm1^2 + bridge.Tdelay^2) / (2*motor.Tm1*bridge.Tdelay) - 1;

Ri.Ki = K1 / Ai;
Ri.Kp = Ri.Ki * Ri.Tpi;

Gi = Ri.Ki * (1 + s*Ri.Tpi) * Li1 / s;

Wi = Gi / (1 + Gi);


%% PI Tune of Speed Controller

Lw = Wi * motor.Kphi * motor.gearbox / m_pole;
Lw1 = Lw / s;
[mag_w1, phase_w1] = bode(Lw1, Rw.wb);

Rw.Tpi = tan(Rw.mphi - 180 - phase_w1) / Rw.wb;

Lw2 = (1+s*Rw.Tpi) * Lw1;
[mag_w2, phase_w2] = bode(Lw2, Rw.wb);

Rw.Ki = 1 / mag_w2;
Rw.Kp = Rw.Ki * Rw.Tpi;

Gw = Rw.Ki * Lw2;

Ww = Gw / (1 + Gw);


%% P Tune of Position Controller

Lt = Ww / s;
[mag_t1, phase_t1] = bode(Lt, Rt.wb);

Rt.Kp = 1/mag_t1;

Gt = Rt.Kp * Lt;

Wt = Gt / (1 + Gt);
