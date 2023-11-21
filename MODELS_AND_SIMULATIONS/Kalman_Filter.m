%% Init

clc
close all

Full_Model_params;
Controller_Tune;


%% State space model
A = [0, motor.gearbox, 0; 0, -motor.B/motor.J, motor.Kphi/motor.J; 0, -motor.Kphi/motor.La, -motor.Ra/motor.La];
B = [0; 0; 1/motor.La];
C = [0, 0, 1; 1, 0, 0];
D = zeros(size(C,1),size(B,2));

Q = [0, 0, 0; 0, 0, 0; 0, 0, 1];
R = 0.01*eye(2);

sys = ss(A,B,C,D);

dsys = c2d(sys, uc.Ts, 'zoh');

[dA, dB, dC, dD] = ssdata(dsys);



%% Kalman filter 
% Q = [0 0 0; 0 0 0; 0 0 1];
% R = eye(2);
% N = zeros(3,2);


% Kf = lqr(A',C',Q,R)

% [Kf, P, E] = lqe(A, Q, C, Q, R, N)


% [Kf_curr,P_curr,E_curr] = lqe(A, Q, C_curr, Q, R);
% 
% [Kf_pos,P_pos,E_pos] = lqe(A, Q, C_pos, Q, R);
