% sets uo the linear model for control design
R = 1.2;
L0 = 0.05;
Ke = 0.6;
Kt = 0.6;
J0 = 0.135;
J = 0.1352;
b = 0;
A = [0 1 0;
     0 -b/J0 Kt/J0;
     0 -Ke/L0 -R/L0];
B = [zeros(2,1); 1/L0];
C = eye(3);
D = [0; 0; 0];

% identifies the matrix sub-blocks of the regular form
A11 = A(1:2 , 1:2);
A12 = A(1:2 , 3);
A21 = A(3 , 1:2);
A22 = A(3, 3);

% values of the natural frequency and damping ratio
wn = 2;
z = 0.95;

% Computes the switching function
M = [wn*wn 2*z*wn]/A12(2,1);
S = [M 1];

% Computes the linear components of the control law
Caphi = -20;
Leq = -inv(S*B)*S*A;
Lrc = Leq + inv(S*B)*Caphi*S;

L = 0.046;
Aper = [0 1 0;
        0 -b/J Kt/J;
        0 -Ke/L -R/L];
Bper = [zeros(2,1); 1/L];

X = [1; 0; 0];
gamma2 = 0.01;