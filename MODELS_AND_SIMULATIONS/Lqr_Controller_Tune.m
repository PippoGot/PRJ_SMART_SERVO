%% Initialization

close all
clc

Model_Parameters;

s = tf('s');

%% State Space Model

% Continuous Time

A = [
     0 motor.gearbox 0; 
     0 -motor.B/motor.J motor.Kphi/motor.J; 
     0 -motor.Kphi/motor.La -motor.Ra/motor.La
    ];
B = [0; 0; 1/motor.La];
C = [1 0 0; 0 0 1];
D = zeros(size(C,1), 1);

SYSC = ss(A, B, C, D);

stsp = [A B; C D];
N = pinv(stsp)*[0; 0; 0; 1; 1];

Nx = N(1:3);
Nu = N(4:end);

% Lqr design

Q = C'*C;
R =  1/5000;

K = lqr(SYSC, Q, R);
% K = lqry(SYSC, 1, R);
Ki = 0;



%% Extended State Space Model 

Ae = [zeros(size(C,1), 2), C; zeros(size(A,1), 2), A];
Be = [zeros(size(C,1), 1); B];
Ce = [zeros(size(C,1), 2), C];
De = zeros(size(Ce,1), size(Be,2));

SYSCe = ss(Ae, Be, Ce, De);


% Lqr design with robust

qi = [10, 0; 0, 10];
Qe = [qi, zeros(2, size(Q,2)); zeros(size(Q,1), 2), Q];

Ke = lqr(SYSCe, Qe, R);

