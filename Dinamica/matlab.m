clc 
clear
close all

syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 q1 q2 q3 q1_d q2_d q3_d Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 g

% Matriz de Rotaciones Iniciales

T01 = [cos(q1) 0 sin(q1) 0; 
       sin(q1) 0 -cos(q1) 0; 
       0 1 0 l1; 
       0 0 0 1];

T12 = [cos(q2) -sin(q2) 0 l2*cos(q2); 
       sin(q2) cos(q2) 0 l2*sin(q2); 
       0 0 1 0; 
       0 0 0 1];

T23 = [cos(q3) -sin(q3) 0 l3*cos(q3); 
       sin(q3) cos(q3) 0 l3*sin(q3); 
       0 0 1 0; 
       0 0 0 1];

% Calculos de Matrices Rotacionales

T02 = T01*T12;
T03 = T01*T12*T23;

% Valores de R01 R02 y R03

R01 = [cos(q1), 0, sin(q1);
      sin(q1), 0, -cos(q1);
      0, 1, 0];

R02 = [cos(q1)*cos(q2), -cos(q1)*sin(q2), sin(q1);
       cos(q2)*sin(q1), -sin(q1)*sin(q2), -cos(q1);
       sin(q2), cos(q2), 0];

R03 = [cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3), - cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2),  sin(q1);
       cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3), - cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2), -cos(q1);
       cos(q2)*sin(q3) + cos(q3)*sin(q2), cos(q2)*cos(q3) - sin(q2)*sin(q3), 0,];

% Jacobianos de Velocidad

Jv1 = [0 0 0;
       0 0 0;
       0 0 0];

Jv2 = [-lc2*cos(q2)*sin(q1) -lc2*cos(q1)*sin(q2) 0;
        lc2*cos(q1)*cos(q2) -lc2*sin(q1)*sin(q2) 0;
        0 (l1+lc2)*cos(q2) 0];

Jv3 = [lc3*sin(q1)*sin(q2)*sin(q3) - lc3*cos(q2)*cos(q3)*sin(q1) - l2*cos(q2)*sin(q1), - l2*cos(q1)*sin(q2) - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2), - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2);
       lc3*sin(q1)*sin(q2)*sin(q3) - lc3*cos(q2)*cos(q3)*sin(q1) - l2*cos(q2)*sin(q1), - l2*cos(q1)*sin(q2) - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2), - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2);
        0, cos(q2)*(l1 + l2) + lc3*cos(q2)*cos(q3) - lc3*sin(q2)*sin(q3), lc3*cos(q2)*cos(q3) - lc3*sin(q2)*sin(q3)];

% Jacobiano de Velocidad Angular

p1 = 1;
p2 = 1;
p3 = 1;

k = [0;0;1];

z0 = k;

z1 = R01*k;

z2 = R02*k;

Jw = [p1*z0, p2*z1, p3*z2];

% Jw =
 
% [0,  sin(q1),  sin(q1)]
% [0, -cos(q1), -cos(q1)]
% [1,        0,        0]

% CALCULO H1 %

RT = R01.';
I1 = [Ixx1 0 0;
      0 Iyy1 0;
      0 0 Izz1];
Ichida = R01*I1*RT;

Jv1T = Jv1.';

Jw1 = [0 0 0;
       0 0 0;
       1 0 0];

Jw1T = Jw1.';

H1 = m1*Jv1T*Jv1 + Jw1T*Ichida*Jw1;

% CALCULO H2 %

RT2 = R02.';
I2 = [Ixx2 0 0;
      0 Iyy2 0;
      0 0 Izz2];
Ichida2 = R02*I2*RT2;

Jv2T = Jv2.';

Jw2 = [0 sin(q1) 0;
       0 -cos(q1) 0;
       1 0 0];

Jw2T = Jw2.';

H2 = m2*Jv2T*Jv2 + Jw2T*Ichida2*Jw2;

% CALCULO H3 %

RT3 = R03.';
I3 = [Ixx3 0 0;
      0 Iyy3 0;
      0 0 Izz3];
Ichida3 = R03*I3*RT3;

Jv3T = Jv3.';

Jw3 = [0 sin(q1) sin(q1);
       0 -cos(q1) -cos(q1);
       1 0 0];

Jw3T = Jw3.';

H3 = m3*Jv3T*Jv3 + Jw3T*Ichida3*Jw3;

%%%%% H FINAL %%%%%

H = H1 + H2 + H3

%%%%% VECTOR DE GRAVEDAD %%%%%

P1 = m1*g*lc1*sin(q1);
P2 = m2*g*(l1*sin(q1)+lc2*sin(q1+q2));

P = P1 + P2;

P_q1 = diff(P,q1);
P_q2 = diff(P,q2);

% g_q = [P_q1;P_q2]

%%%%% CALCULO DE CORIOLIS %%%%%

h11 = H(1,1);
h12 = H(1,2); 
h13 = H(1,3); 

h21 = H(2,1);
h22 = H(2,2);
h23 = H(2,3);

h31 = H(3,1);
h32 = H(3,2);
h33 = H(3,3);

% PRIMERA COLUMNA DE LA MATRIZ

% Calculos para calcular C11

C111 = 0.5 * (diff(h11,q1) +  diff(h11,q1) -  diff(h11,q1)); 
C211 = 0.5 * (diff(h11,q2) +  diff(h12,q1) -  diff(h21,q1)); 
C311 = 0.5 * (diff(h11,q3) +  diff(h13,q1) -  diff(h31,q1)); 

C11 = (C111*q1_d) + (C211*q2_d) + (C311*q3_d);

% Calculos para calcular C12

C121 = 0.5 * (diff(h12,q1) +  diff(h11,q2) -  diff(h12,q1)); 
C221 = 0.5 * (diff(h12,q2) +  diff(h12,q2) -  diff(h22,q1)); 
C321 = 0.5 * (diff(h12,q3) +  diff(h13,q2) -  diff(h32,q1)); 

C12 = (C121*q1_d) + (C221*q2_d) + (C321*q3_d);

% Calculos para calcular C13

C131 = 0.5 * (diff(h13,q1) +  diff(h11,q3) +  diff(h13,q1)); 
C231 = 0.5 * (diff(h13,q2) +  diff(h12,q3) +  diff(h23,q1)); 
C331 = 0.5 * (diff(h13,q3) +  diff(h13,q3) +  diff(h33,q1)); 

C13 = (C131*q1_d) + (C231*q2_d) + (C331*q3_d); 

% SEGUNDA COLUMNA DE LA MATRIZ

% Calculos para calcular C21

C112 = 0.5 * (diff(h21,q1) + diff(h21,q1) - diff(h11,q2)); 
C212 = 0.5 * (diff(h21,q2) + diff(h22,q1) - diff(h21,q2)); 
C312 = 0.5 * (diff(h21,q3) + diff(h23,q1) - diff(h31,q2));

C21 = (C112*q1_d) + (C212*q2_d) + (C312*q3_d);

% Calculos para calcular C22

C122 = 0.5 * (diff(h22,q1) + diff(h21,q2) - diff(h12,q2)); 
C222 = 0.5 * (diff(h22,q2) + diff(h22,q2) - diff(h22,q2)); 
C322 = 0.5 * (diff(h22,q3) + diff(h23,q2) - diff(h32,q2)); 

C22 = (C122*q1_d) + (C222*q2_d) + (C322*q3_d);

% Calculos para calcular C23

C132 = 0.5 * (diff(h23,q1) + diff(h21,q3) - diff(h13,q2)); 
C232 = 0.5 * (diff(h23,q2) + diff(h22,q3) - diff(h23,q2)); 
C332 = 0.5 * (diff(h23,q3) + diff(h23,q3) - diff(h33,q2)); 

C23 = (C132*q1_d) + (C232*q2_d) + (C332*q3_d);

% TERCERA COLUMNA DE LA MATRIZ

% Calculos para calcular C31

C113 = 0.5 * (diff(h31,q1) + diff(h31,q1) + diff(h11,q3)); 
C213 = 0.5 * (diff(h31,q2) + diff(h32,q1) + diff(h21,q3)); 
C313 = 0.5 * (diff(h31,q3) + diff(h33,q1) + diff(h31,q3));

C31 = (C113*q1_d) + (C213*q2_d) + (C313*q3_d);

% Calculos para calcular C32

C123 = 0.5 * (diff(h32,q1) + diff(h31,q2) + diff(h12,q3)); 
C223 = 0.5 * (diff(h32,q2) + diff(h32,q2) + diff(h22,q3)); 
C323 = 0.5 * (diff(h32,q3) + diff(h33,q2) + diff(h32,q3)); 

C32 = (C123*q1_d) + (C223*q2_d) + (C323*q3_d);

% Calculos para calcular C33; 

C133 = 0.5 * (diff(h33,q1) + diff(h31,q3) + diff(h13,q3));
C233 = 0.5 * (diff(h33,q2) + diff(h32,q3) + diff(h23,q3));
C333 = 0.5 * (diff(h33,q3) + diff(h33,q3) + diff(h33,q3));

C33 = (C133*q1_d) + (C233*q2_d) + (C333*q3_d);


%%%%% MATRIZ C FINAL %%%%%

C = [C11 C12 C13; C21 C22 C23; C31 C32 C33]
