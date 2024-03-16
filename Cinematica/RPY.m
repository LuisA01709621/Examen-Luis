clc
clear all
theta1 = pi/2;
theta2 = pi/3;
d3 = 0.75;
l1 = 0.5;

R = [cos(theta1)*cos(theta2 + pi/2), -cos(theta1)*sin(theta2 + pi/2),  sin(theta1), l1 + d3*cos(theta1)*cos(theta2 + pi/2);           -sin(theta2 + pi/2),             -cos(theta2 + pi/2),            0,                 -d3*sin(theta2 + pi/2);
    cos(theta2 + pi/2)*sin(theta1), -sin(theta1)*sin(theta2 + pi/2), -cos(theta1),      d3*cos(theta2 + pi/2)*sin(theta1);
                             0,                               0,            0,                                      1];
 


%Extraer los elementos individuales de la matriz de rotación
r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

% Calcular pitch (θ)
pitch = atan2(-r31, sqrt(r11^2 + r21^2));

% Calcular roll (φ)
roll = atan2(r32/cos(pitch), r33/cos(pitch));

% Calcular yaw (ψ)
yaw = atan2(r21, r11);

% Ajustar los ángulos para que estén dentro del rango especificado
while roll < -pi
    roll = roll + 2*pi;
end
while roll > pi
    roll = roll - 2*pi;
end

while pitch < -pi/2
    pitch = pitch + pi;
end
while pitch > pi/2
    pitch = pitch - pi;
end

while yaw < -pi
    yaw = yaw + 2*pi;
end
while yaw > pi
    yaw = yaw - 2*pi;
end

% Mostrar los ángulos calculados
disp('Ángulos calculados:');
disp(['Roll (φ): ', num2str(roll)]);
disp(['Pitch (θ): ', num2str(pitch)]);
disp(['Yaw (ψ): ', num2str(yaw)]);
