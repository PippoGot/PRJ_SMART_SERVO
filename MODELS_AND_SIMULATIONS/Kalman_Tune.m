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
Q = diag([0, 0, 1, 0]);
R = diag([AS5600.q_rad^2/12, INA219.current_LSB^2/12 + 1e-6]);


%% Conversion to Discrete Time

Ad = expm(Ak*uc.Ts);
Bd = integral(@(t) expm(Ak.*t), 0, uc.Ts, 'ArrayValued', true)*Bk;

O = [Ck; Ck*Ad; Ck*Ad^2; Ck*Ad^3];
observable = rank(O)

Qd = integral(@(t) expm(Ak.*t)*Q*expm(Ak'.*t), 0, uc.Ts, 'ArrayValued', true);