%% Initialization

close all
clc

Full_Model_params;
Controller_Tune;


%% State Space Model

% Continuous Time
A = [0 motor.gearbox 0; 0 -motor.B/motor.J motor.Kphi/motor.J; 0 -motor.Kphi/motor.La -motor.Ra/motor.La];
B = [0; 0; 1/motor.La];
C = [1 0 0; 0 0 1];
D = zeros(2, 1);

SYSC = ss(A, B, C, D);


% Discrete Time
SYSD = ss(A, B, C, D, uc.Ts);

% Noise Matrices

Q = diag([1e-6, 1e-6, 1e-6]);
R = diag([0, 1e-3]);


%% Observer Gain and State Space

poles = 4*eigs(A) + [0; 0; -200];

L = place(A', C', poles)';

Ao = A - L*C;
Bo = [B L];
Co = eye(3);
Do = zeros(3, 3);