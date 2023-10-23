%% Initialization

clear
close all 
clc


%% Conversion Constants

conv.rpm__to__rad_s = 2*pi/60;
conv.g_cm2__to__kg_m2 = 1e-7;
conv.kgf_cm__to__Nm = 0;


%% Power Parameters

pwr.Vcc = 5;        % supply voltage of motor and power components          [V]
pwr.Vuc = 3.3;      % supply voltage of microcontroller                     [V]


%% H-Bridge Parameters (From Datasheet)

% PMOS
PMOS.Vth = -0.7;        % threshold voltage of the P-MOSFETs                [V]
PMOS.Rdson = 0.09;      % resistance of the MOSFETs channel when on         [ohm]
PMOS.Vgson = -4.5;      % gate-source voltage for Rdson                     [V]
PMOS.Idson = -2.2;      % drain-source current for Rdson                    [A]

PMOS.Ciss = 610;        % input capacitance                                 [pF]
PMOS.Crss = 170;        % reverse transfer capacitance                      [pF]
PMOS.Coss = 310;        % output capacitance                                [pF]

% NMOS
NMOS.Vth = 0.7;         % threshold voltage of the N-MOSFETs                [V]
NMOS.Rdson = 0.05;      % resistance of the MOSFETs channel when on         [ohm]
NMOS.Vgson = 4.5;       % gate-source voltage for Rdson                     [V]
NMOS.Idson = 2.6;       % drain-source current for Rdson                    [A]

NMOS.Ciss = 660;        % input capacitance                                 [pF]
NMOS.Crss = 140;        % reverse transfer capacitance                      [pF]
NMOS.Coss = 280;        % output capacitance                                [pF]

% Body Diode
diode.Vd = 1;           % diode forward voltage                             [V]
diode.Id = 2.5;         % diode source current                              [A]


%% Inverter MOSFET Parameters (From Datasheet)

AO3400.Vth = 1.4;       % threshold voltage of the MOSFET                   [V]
AO3400.Rdson = 0.04;    % resistance of the MOSFET channel when on          [ohm]
AO3400.Vgson = 4.5;     % gate-source voltage for Rdson                     [V]
AO3400.Idson = 5;       % drain-source current for Rdson                    [A]

AO3400.Ciss = 1050;     % input capacitance                                 [pF]
AO3400.Crss = 77;       % reverse transfer capacitance                      [pF]
AO3400.Coss = 99;       % output capacitance                                [pF]


%% Motor Parameters

motor.Ra = 2.4;                 % motor armature resistance                 [ohm]       TO MEASURE
motor.La = 2e-9;                % motor armature inductance                 [H]         TO MEASURE


motor.Vn = 5;                   % motor rated voltage                       [V]         FROM DS
motor.wmo = 12000;              % motor no-load speed                       [rpm]       FROM DS
motor.Imo = 0.14;               % motor no-load current                     [A]         FROM DS

motor.tl = 0.98;                % motor rated load                          [mN*m]      FROM DS
motor.wn = 10600;               % motor load speed                          [rpm]       FROM DS
motor.In = 0.42;                % motor load current                        [A]         FROM DS

motor.ts = 4.5;                 % motor stall torque                        [mN*m]      INDIRECTLY FROM DS

motor.J = 1.2;                  % motor inertia                             [g*cm^2]    TO ESTIMATE
motor.B = 0;                    % motor friction                            []          TO ESTIMATE

motor.gearbox = 11/(61*36);     % gearbox ratio                             [#]         COUNTED

% motor geometry constant
motor.Kphi = motor.Vn / (motor.wmo * conv.rpm__to__rad_s);                 %[V*s] 

% motor electric time constant              
motor.Te = motor.La / motor.Ra;                                            %[s]

% motor electromechanical time constant
motor.Tm1 = (motor.J * conv.g_cm2__to__kg_m2 * motor.Ra) / motor.Kphi^2;   %[s]


%% Voltage Converter Parameters

vc.Kc = 5;      % voltage gain                                              [V]
vc.wb = 2e6;    % deadband frequency                                        [rad_s]















