%% s Variable and Options Definition

s = tf('s');
options = pidtuneOptions('PhaseMargin', 90);

%% Transfer Functions

% H-Bridge
PE = bridge.gain / (1 + s*bridge.Tdelay);

% Motor Current
M1 = (motor.B + s*motor.J) / ((motor.B + s*motor.J)*(motor.Ra + s*motor.La) + motor.Kphi^2);

% Motor Speed
M2 = motor.Kphi * motor.gearbox / (motor.B + s*motor.J);


% Current Sensor Filter
CF = cf.w^2 / (s^2 + 2*cf.csi*cf.w*s + cf.w^2);

% Position Sensor Filter
PF = sf.w^2 / (s^2 + 2*sf.csi*sf.w*s + sf.w^2);

% Speed Sensor Filter
SF = 500^2 / (s^2 + 2*sf.csi*500*s + 500^2) * PF;


%% PI Tune of Current Controller

Pi = PE * M1;                  % Open Loop Current 
[Ri, INFOi] = pidtune(Pi, 'PI');    % Autotune

Ci = Ri.Kp + Ri.Ki / s              % Current Controller

Li = Ci * Pi;                       % Open Loop + Controller
Gi = Li / (1 + Li);                 % Closed Loop Current


%% PI Tune of Speed Controller

Pw = Gi * M2;                  % Open Loop Speed  
[Rw, INFOw] = pidtune(Pw, 'PI');    % Autotune

Cw = Rw.Kp + Rw.Ki / s             % Speed Controller

Lw = Cw * Pw;                       % Open Loop + Controller
Gw = Lw / (1 + Lw);                 % Closed Loop Speed


%% P Tune of Position Controller

Pt = PF * Gw / s;                   % Open Loop Position
[Rt, INFOt] = pidtune(Pt, 'PIDF');    % Autotune

Ct = Rt.Kp + Rt.Ki / s             % Position Controller

Lt = Ct * Pt;                       % Open Loop + Controller
Gt = Lt / (1 + Lt);                 % Closed Loop Position

