%% State Space Model

% Continuous Time SS Matrices
Ak = [  0   motor.gearbox           0           0                   ; 
        0   -motor.B/motor.J        -1/motor.J  motor.Kphi/motor.J  ; 
        0   0                       0           0                   ;
        0   -motor.Kphi/motor.La    0           -motor.Ra/motor.La  ];
Bk = [0; 0; 0; 1/motor.La];
Ck = [1 0 0 0; 0 0 0 1];
Dk = zeros(2, 1);


%% Noise Matrices
Q = diag([0, 1, motor.tau, 0]);
R = diag([AS5600.q_rad^2/12, 1]);
