%% Initialization

close all
clc

Full_Model_params;
s = tf('s');


%% PI Tune of Current Controller

Ki1 = (motor.Tm1^2 + vc.Tc^2) / (2*motor.Tm1*vc.Tc) - 1;

Ri.Tpi = motor.Te;
Ri.Ki = Ki1 * motor.Ra / (vc.Kc * motor.Tm1);
Ri.Kp = Ri.Tpi * Ri.Ki;

Ai = Ri.Ki * vc.Kc * motor.Tm1 / motor.Ra;
Li = Ai / ((1 + s * motor.Tm1) * (1 + s * vc.Tc));
Gi = Li / (1 + Li);


%% PI Tune of Speed Controller

Rw.tr = 0.05;
Rw.wb = 2.2 / Rw.tr;
Rw.mphi = 65;

Rw.Tpi = -tan(Rw.mphi) / Rw.wb;


Lw1 = Gi * motor.Kphi * (1 + s * Rw.Tpi) / (s^2 * motor.J);
[magnitude, phase] = bode(Lw1, Rw.wb);

Rw.Ki = 1 / magnitude;
Rw.Kp = Rw.Tpi * Rw.Ki;

Lw = Rw.Ki * Lw1;
Gw = Lw / (1 + Lw);


%% P Tune of Position Controller

Rp.tr = 0.0005;
Rp.wb = 2.2 / Rp.tr;
Rp.mphi = Rw.mphi;

Rp.Tpi = -tan(Rp.mphi) / Rp.wb;


Lp1 = Gw * (1 + s * Rp.Tpi) / s^2;
[magnitude, phase] = bode(Lp1, Rp.wb);

Rp.Ki = 1 / magnitude;
Rp.Kp = Rp.Tpi * Rp.Ki;

Lp = Rp.Ki * Lp1;
Gp = Lp / (1 + Lp);





