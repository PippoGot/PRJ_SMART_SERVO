%% Conversion Constants

conv.g_cm2__to__kg_m2 = 1e-7;                   %                           [g/cm^2] -> [kg/m^2]
conv.kgf_cm__to__Nm = 0;                        %                           [kgf*cm] -> [N*m]

conv.deg__to__rad = pi/180;                     %                           [deg] -> [rad]
conv.rad__to__deg = 1/conv.deg__to__rad;        %                           [rad] -> [deg]

conv.rpm__to__rad_s = 2*pi/60;                  %                           [rpm] -> [rad/s]
conv.rad_s__to__rpm = 1/conv.rpm__to__rad_s;    %                           [rad/s] -> [rpm]


%% Saturation Values

sat.w = 50 * conv.rpm__to__rad_s;       % saturation speed                  [rad/s]
sat.I = 1.5;                            % saturation current                [I]
sat.d = 1;                              % saturation duty-cycle             [#]


%% Power Parameters

pwr.Vcc = 5;        % supply voltage of motor and h-bridge                  [V]


%% Microcontroller Parameters

% General
uc.pwr = 3.3;           % mictrocontroller supply                           [V]
uc.fclk = 64e6;         % internal clock frequency                          [Hz]
uc.ftim = uc.fclk / 2;  % timer clock frequency                             [Hz]

uc.Ts = 1e-3;           % sampling time                                     [s]


% PWM
uc.fpwm = 1e6;                  % PWM frequency                             [Hz]
uc.Tpwm = 1 / uc.fpwm;          % PWM period                                [s]

uc.pwm_steps = 1e3;             % number of PWM steps per period            [#]
uc.d_step = 1 / uc.pwm_steps;   % smallest duty cycle variation             [%]

uc.pwm_psc = uc.fclk / (uc.pwm_steps * uc.fpwm) - 1;    % prescaler         [#]


%% H-Bridge Parameters (From IRF7307 Datasheet)

% PMOS
PMOS.Vth = -0.7;        % threshold voltage of the P-MOSFETs                [V]

PMOS.Rdson = 0.09;      % resistance of the MOSFETs channel when on         [Ohm]
PMOS.Vgson = -4.5;      % gate-source voltage for Rdson                     [V]
PMOS.Idson = -2.2;      % drain-source current for Rdson                    [A]

PMOS.Ciss = 610;        % input capacitance                                 [pF]
PMOS.Crss = 170;        % reverse transfer capacitance                      [pF]
PMOS.Coss = 310;        % output capacitance                                [pF]


% NMOS
NMOS.Vth = 0.7;         % threshold voltage of the N-MOSFETs                [V]

NMOS.Rdson = 0.05;      % resistance of the MOSFETs channel when on         [Ohm]
NMOS.Vgson = 4.5;       % gate-source voltage for Rdson                     [V]
NMOS.Idson = 2.6;       % drain-source current for Rdson                    [A]

NMOS.Ciss = 660;        % input capacitance                                 [pF]
NMOS.Crss = 140;        % reverse transfer capacitance                      [pF]
NMOS.Coss = 280;        % output capacitance                                [pF]


% Body Diode
diode.Vd = 1;           % diode forward voltage                             [V]
diode.Id = 2.5;         % diode source current                              [A]


% Transfer Function Parameters
bridge.fsw = uc.fpwm;                   % switching frequency               [Hz]
bridge.Tdelay = 1 / uc.Ts;              % bridge delay time                 [s]

bridge.gain = pwr.Vcc;                  % bridge voltage gain               [V]

bridge.R = PMOS.Rdson + NMOS.Rdson;     % bridge path on resistance         [Ohm]


%% Inverter MOSFET Parameters (From AO3400 Datasheet)

AO3400.Vth = 1.4;       % threshold voltage of the MOSFET                   [V]
AO3400.Rdson = 0.04;    % resistance of the MOSFET channel when on          [Ohm]
AO3400.Vgson = 4.5;     % gate-source voltage for Rdson                     [V]
AO3400.Idson = 5;       % drain-source current for Rdson                    [A]

AO3400.Ciss = 1050;     % input capacitance                                 [pF]
AO3400.Crss = 77;       % reverse transfer capacitance                      [pF]
AO3400.Coss = 99;       % output capacitance                                [pF]


%% Motor Parameters
% !!! Keep in mind the datasheet (DS) is not reliable !!!

% For Model
motor.Ra = 2;                           % motor armature resistance         [Ohm]       MEASURED
motor.La = 7e-3;                        % motor armature inductance         [H]         MEASURED

motor.Kphi = 1.8e-3;                    % torque/back emf constant          [V*s]       ESTIMATED

motor.J = 1.0624e-8;                    % motor inertia                     [Kg*m^2]    ESTIMATED
motor.B = 1.4969e-8;                    % motor friction                    [N*m*s]     ESTIMATED
motor.tau = 7.5179e-5;                  % motor static friction             [N*m]       ESTIMATED

motor.gearbox = 11/(61*36);             % gearbox ratio                     [#]         COUNTED

% No Load Parameters
motor.wmo = 12000;      % motor no-load speed                               [rpm]       FROM DS
motor.Imo = 0.14;       % motor no-load current                             [A]         FROM DS

% Rated Parameters
motor.Vn = 5;           % motor rated voltage                               [V]         FROM DS
motor.tl = 0.98;        % motor rated load                                  [mN*m]      FROM DS
motor.wn = 10600;       % motor load speed                                  [rpm]       FROM DS
motor.In = 0.42;        % motor load current                                [A]         FROM DS

motor.ts = 4.5;         % motor stall torque                                [mN*m]      INDIRECTLY FROM DS


% motor electric time constant              
motor.Te = motor.La / motor.Ra;                             %               [s]

% motor electromechanical time constant
motor.Tm1 = (motor.J * motor.Ra) / motor.Kphi^2;            %               [s]

% motor mechanical time constant
motor.Tm = motor.J / motor.B;                               %               [s]


%% Sensors Parameters

% Current/Voltage Sensor
INA219.Rs = 0.05;                       % shunt resistance                  [Ohm]
INA219.Imax = 3;                        % maximum expected current          [A]

INA219.current_LSB = INA219.Imax/2^16;  % current quantization step         [A]
INA219.voltage_LSB = 4e-3;              % bus voltage quantization step     [V]

% Magnetic Encoder 
AS5600.q_deg = 360 / 4096;              % quantization step of position     [deg]
AS5600.q_rad = 2 * pi / 4096;           % quantization step of position     [rad]


%% Low Pass Filters Parameters

% Speed Filter
sf.csi = 1/sqrt(2);                     % speed filter dampening coeff.     [#]
sf.w = 100;                             % speed filter cutoff frequency     [rad/s]

% Current Filter
cf.csi = 1/sqrt(2);                     % current filter dampening coeff.   [#]
cf.w = 500;                             % current filter cutoff frequency   [rad/s]

% Voltage Filter
vf.csi = 1/sqrt(2);                     % voltage filter dampening coeff.   [#]
vf.w = 500;                             % voltage filter cutoff frequency   [rad/s]


