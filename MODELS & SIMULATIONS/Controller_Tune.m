%% Initialization

close all
clc

Full_Model_params;

s = tf('s');

%% PI Tune of Current Controller

% current requirements
wc = vc.wbi/10;


% open loop current tf
Gi = s * (vc.Kc * motor.Tm1 / motor.Ra) / ((1 + s / vc.wbi)*(1 + s * motor.Tm1)*(1 + s * motor.Te));

figure(1)
bode(Gi)

% current pidtune PI
Ri.Tpi = motor.Te;      % set to have a cancellation

[mag, phase] = bode(Gi * (1 + s * Ri.Tpi) / s, wc); % get magnitude

Ri.Ki = 1/mag;          % integral coefficient
Ri.Kp = Ri.Tpi*Ri.Ki;   % proportional coefficient

Li = (Ri.Ki * (1 + s * Ri.Tpi) / s) * Gi;
Wi = Li/(1 + Li);

figure(2)
% margin(Li)
bode(Li, Wi)


%% PI Tune of Speed Controller

% speed requirements
tr = 2e-3;              % rise time                                         [s]
wb = 2.2 / tr;          % crossing frequency                                [rad_s]
m_phi = 65;             % minimum phase margin                              [deg]

% open loop speed tf
Gw = Wi * motor.Kphi / (s * motor.J);

figure(3)
bode(Gw)

% speed pidtune PI
Rw.Tpi = - tan(m_phi) / wb;

[mag, phase] = bode((Gw * (1 + s * Rw.Tpi) / s), wb);    % get magnitude

Rw.Ki = 1/mag;          % integral coefficient
Rw.Kp = Rw.Tpi * Rw.Ki;   % proportional coefficient

Lw = (Rw.Ki * (1 + s * Rw.Tpi) / s) * Gw;

figure(4)
margin(Lw)








