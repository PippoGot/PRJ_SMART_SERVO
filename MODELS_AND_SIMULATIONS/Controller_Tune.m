%% Initialization

close all
%clc

Full_Model_params;

s = tf('s');

%% PI Tune of Current Controller

% requirements
tr = 1e-4;              % rise time                                         [s]
wc = 2.2 / tr;          % crossing frequency                                [rad_s]

% open loop current tf
Gi = s * (vc.Kc * motor.Tm1 / motor.Ra) / ((1 + s / vc.wb)*(1 + s * motor.Tm1)*(1 + s * motor.Te));

%figure(1)
%bode(Gi)

% current pidtune PI
Ri.Tpi = motor.Tm1;     % set to have a cancellation

[mag, phase] = bode(Gi * (1 + s * Ri.Tpi) / s, wc); % get magnitude

Ri.Ki = 1/mag;          % integral coefficient
Ri.Kp = Ri.Tpi*Ri.Ki;   % proportional coefficient

Li = (Ri.Ki * (1 + s * Ri.Tpi) / s) * Gi;

figure(2)
margin(Li)


%% PI Tune of Speed Controller

% open loop speed tf
Gw = Li/(1+Li) * motor.Kphi / (s * motor.J);

%figure(3)
%bode(Gw)

% speed pidtune PI
Rw.Tpi = motor.Tm1;

[mag, phase] = bode(Gw, wc);    % get magnitude

Rw.Ki = 1/mag;          % integral coefficient
Rw.Kp = Rw.Tpi*Rw.Ki;   % proportional coefficient

Lw = (Rw.Ki * (1 + s * Rw.Tpi) / s) * Gw;

figure(4)
margin(Lw)



