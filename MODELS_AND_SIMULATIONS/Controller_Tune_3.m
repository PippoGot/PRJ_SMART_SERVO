%% s Variable Definition

s = tf('s');

%% Transfer Functions

% H-Bridge
PE = bridge.gain / (1 + s*bridge.Tdelay);

% Mechanincal Part
Me = 1 / (motor.B + s*motor.J);

% Electrical Part
El = 1 / (motor.Ra + s*motor.La);

% Current Transfer Function
CL = PE * El / (1 + El * Me * motor.Kphi^2);

% Speed Sensor Filter
SF = sf.w^2*s / (s^2 + 2*sf.csi*sf.w*s + sf.w^2);

% Current Sensor Filter
CF = cf.w^2 / (s^2 + 2*cf.csi*cf.w*s + cf.w^2);


%% Current PI Tune

Ri.wb = 2 / Ri.tr;
Ri.mphi = 1.04 - 0.8 * Ri.mp;

Li1 = CL;
[mag1_i, phase1_i] = bode(Li1, Ri.wb);

Ri.phi0 = Ri.mphi - pi - phase1_i * conv.deg__to__rad;
Ri.phi1 = Ri.wb * uc.Ts / 2;
[mag2_i, phase2_i] = bode(CF, Ri.wb);
Ri.phi2 = phase2_i * conv.deg__to__rad;
Ri.phi = Ri.phi0 + Ri.phi1 + Ri.phi2;
Ri.M = 1 / mag1_i;

Ri.Kp = Ri.M * cos(Ri.phi);
Ri.Ki = -Ri.Kp * Ri.wb * tan(Ri.phi);

Ci = Ri.Kp + Ri.Ki / s;
Li = Ci * Li1;
Gi = Li / (1 + Li);


%% Speed PI Tune

Rw.wb = 2 / Rw.tr;
Rw.mphi = 1.04 - 0.8 * Rw.mp;

Lw1 = Gi * Me * motor.Kphi * motor.gearbox;
[mag1_w, phase1_w] = bode(Lw1, Rw.wb);

Rw.phi0 = Rw.mphi - pi - phase1_w * conv.deg__to__rad;
Rw.phi1 = Rw.wb * uc.Ts / 2;
[mag2_w, phase2_w] = bode(SF, Rw.wb);
Rw.phi2 = phase2_w * conv.deg__to__rad;
Rw.phi = Rw.phi0 + Rw.phi1 + Rw.phi2;
Rw.M = 1 / mag1_w;

Rw.Kp = Rw.M * cos(Rw.phi);
Rw.Ki = -Rw.Kp * Rw.wb * tan(Rw.phi);

Cw = Rw.Kp + Rw.Ki / s;
Lw = Cw * Lw1;
Gw = Lw / (1 + Lw);


%% Position PI Tune

Rt.wb = 2 / Rt.tr;
Rt.mphi = 1.04 - 0.8 * Rt.mp;

Lt1 = Gw / s;
[mag1_t, phase1_t] = bode(Lt1, Rt.wb);

Rt.phi0 = Rt.mphi - pi - phase1_t * conv.deg__to__rad;
Rt.phi1 = Rt.wb * uc.Ts / 2;
Rt.phi = Rt.phi0 + Rt.phi1;
Rt.M = 1 / mag1_t;

Rt.Kp = Rt.M * cos(Rt.phi);
Rt.Ki = -Rt.Kp * Rt.wb * tan(Rt.phi);

Ct = Rt.Kp + Rt.Ki / s;
Lt = Ct * Lw1;
Gt = Lt / (1 + Lt);



