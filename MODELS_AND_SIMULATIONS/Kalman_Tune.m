%% State Space Model

% Continuous Time
A = [
    0 motor.gearbox 0 0; 
    0 -motor.B/motor.J -1/motor.J motor.Kphi/motor.J; 
    0 0 0 0;
    0 -motor.Kphi/motor.La 0 -motor.Ra/motor.La
    ];

B = [0; 0; 0; 1/motor.La];
C = [1 0 0 0; 0 0 0 1];
D = zeros(2, 1);

SYSC = ss(A, B, C, D);

