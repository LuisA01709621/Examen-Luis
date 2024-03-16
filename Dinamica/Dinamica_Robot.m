clc; clear all; close all
%% Simbolicos
syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 q1 q2 q3 q1_d q2_d q3_d I_xx1 I_yy1 I_zz1 I_xx2 I_yy2 I_zz2 I_xx3 I_yy3 I_zz3 g

%% Matriz de Rotaciones Iniciales

T_01 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 -cos(q1) 0; 0 1 0 l1; 0 0 0 1];

T_12 = [cos(q2) -sin(q2) 0 l2*cos(q2); sin(q2) cos(q2) 0 l2*sin(q2); 0 0 1 0; 0 0 0 1];

T_23 = [cos(q3) -sin(q3) 0 l3*cos(q3); sin(q3) cos(q3) 0 l3*sin(q3); 0 0 1 0; 0 0 0 1];

T_02 = T_01*T_12;
T_03 = T_01*T_12*T_23;

R_01 = [cos(q1), 0, sin(q1);sin(q1), 0, -cos(q1); 0, 1, 0];

R_02 = [cos(q1)*cos(q2), -cos(q1)*sin(q2), sin(q1); cos(q2)*sin(q1), -sin(q1)*sin(q2), -cos(q1); sin(q2), cos(q2), 0];

R_03 = [cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3), - cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2),  sin(q1); cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3), - cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2), -cos(q1); cos(q2)*sin(q3) + cos(q3)*sin(q2), cos(q2)*cos(q3) - sin(q2)*sin(q3), 0,];

%% Jacobianos Velocidad

Jv1 = zeros(3);
Jv2 = [-lc2*cos(q2)*sin(q1) -lc2*cos(q1)*sin(q2) 0; lc2*cos(q1)*cos(q2) -lc2*sin(q1)*sin(q2) 0; 0 (l1+lc2)*cos(q2) 0];
Jv3 = [lc3*sin(q1)*sin(q2)*sin(q3) - lc3*cos(q2)*cos(q3)*sin(q1) - l2*cos(q2)*sin(q1), - l2*cos(q1)*sin(q2) - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2), - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2); lc3*sin(q1)*sin(q2)*sin(q3) - lc3*cos(q2)*cos(q3)*sin(q1) - l2*cos(q2)*sin(q1), - l2*cos(q1)*sin(q2) - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2), - lc3*cos(q1)*cos(q2)*sin(q3) - lc3*cos(q1)*cos(q3)*sin(q2); 0, cos(q2)*(l1 + l2) + lc3*cos(q2)*cos(q3) - lc3*sin(q2)*sin(q3), lc3*cos(q2)*cos(q3) - lc3*sin(q2)*sin(q3)];

%% Jacobiano Velocidad Angular
p1 = 1; p2 = 1; p3 = 1;
k = [0;0;1];

z0 = k;
z1 = R_01*k;
z2 = R_02*k;
Jw = [p1*z0, p2*z1, p3*z2];
%% H1

RT = R_01.';
I1 = [I_xx1 0 0; 0 I_yy1 0; 0 0 I_zz1];
I = R_01*I1*RT;

Jv1T = Jv1.';

Jw1 = [0 0 0; 0 0 0; 1 0 0];

Jw1T = Jw1.';
H1 = m1*Jv1T*Jv1 + Jw1T*I*Jw1;
%% H2

RT2 = R_02.';
I2 = [I_xx2 0 0; 0 I_yy2 0; 0 0 I_zz2];
I_2 = R_02*I2*RT2;

Jv2T = Jv2.';

Jw2 = [0 sin(q1) 0; 0 -cos(q1) 0; 1 0 0];

Jw2T = Jw2.';
H2 = m2*Jv2T*Jv2 + Jw2T*I_2*Jw2;

%% H3

RT3 = R_03.';
I3 = [I_xx3 0 0; 0 I_yy3 0; 0 0 I_zz3];
I_3 = R_03*I3*RT3;

Jv3T = Jv3.';

Jw3 = [0 sin(q1) sin(q1); 0 -cos(q1) -cos(q1); 1 0 0];

Jw3T = Jw3.';

H3 = m3*Jv3T*Jv3 + Jw3T*I_3*Jw3;
%% H

H = H1 + H2 + H3;
disp('H:')
disp(H)
%% GRAVEDAD
P1=m1*g*0;
P2=m2*g*(lc2*cos(q2)*sin(q1));
P3=m3*g*(l2*cos(q2)*sin(q1)+lc3*cos(q2)*cos(q3)*sin(q1)-lc3*sin(q1)*sin(q2)*sin(q3));
P=P1+P2+P3;

P1_d=diff(P,q1);    P2_d=diff(P,q2);    P3_d=diff(P,q3);
gq=[P1_d;P2_d;P3_d];
disp('Gq:')
disp(gq);
%% CORIOLIS

h11 = H(1,1);   h21 = H(2,1);   h31 = H(3,1);
h12 = H(1,2);   h22 = H(2,2);   h32 = H(3,2);
h13 = H(1,3);   h23 = H(2,3);   h33 = H(3,3);

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


%% MATRIZ C

C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
disp('C:')
disp(C)
