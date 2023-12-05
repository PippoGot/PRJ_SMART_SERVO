%% Initialization

close all
clc

Full_Model_params;
Controller_Tune;


%% State Space Model

A = [0 motor.gearbox 0; 0 -motor.B/motor.J motor.Kphi/motor.J; 0 -motor.Kphi/motor.La -motor.Ra/motor.La];
B = [0; 0; 1/motor.La];
C = [1 0 0; 0 0 1];
D = zeros(2, 1);


%% Observer Gain and State Space

poles = 2*eigs(A) + [0; 0; -500];

L = place(A', C', poles)';

Ao = A - L*C;
Bo = [B L];
Co = eye(3);
Do = zeros(3, 3);