%% State Space Model

% Continuous Time SS Matrices
Ak = [  0   motor.gearbox           0           0                   ; 
        0   -motor.B/motor.J        -1/motor.J  motor.Kphi/motor.J  ; 
        0   0                       0           0                   ;
        0   -motor.Kphi/motor.La    0           -motor.Ra/motor.La  ];
Bk = [0; 0; 0; 1/motor.La];
Ck = [1 0 0 0; 0 0 0 1];
Dk = zeros(2, 1);

KSYSC = ss(Ak, Bk, Ck, Dk);

KSYSD = c2d(KSYSC, uc.Ts);


%% Noise Matrices
Q = diag([2*AS5600.q_rad, 0, 10*motor.tau, 2*INA219.current_LSB]);
R = diag([AS5600.q_rad^2/12, INA219.current_LSB^2/12+1e-3]);
