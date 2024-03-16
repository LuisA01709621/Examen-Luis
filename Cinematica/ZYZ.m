clc
clear all
theta1 = pi/2;
theta2 = pi/3;
d3 = 0.75;
l1 = 0.5;

R = [cos(theta1)*cos(theta2 + pi/2), -cos(theta1)*sin(theta2 + pi/2),  sin(theta1), l1 + d3*cos(theta1)*cos(theta2 + pi/2);           -sin(theta2 + pi/2),             -cos(theta2 + pi/2),            0,                 -d3*sin(theta2 + pi/2);
    cos(theta2 + pi/2)*sin(theta1), -sin(theta1)*sin(theta2 + pi/2), -cos(theta1),      d3*cos(theta2 + pi/2)*sin(theta1);
                             0,                               0,            0,                                      1];

% Calcular beta (β)
beta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
beta = ajustarRango(beta, -pi/2, pi/2);

% Calcular alpha (α)
alpha = atan2(R(2,3)/sin(beta), R(1,3)/sin(beta));
alpha = ajustarRango(alpha, -pi, pi);

% Calcular gamma (γ)
gamma = atan2(R(3,2)/sin(beta), -R(3,1)/sin(beta));
gamma = ajustarRango(gamma, -pi, pi);

% Mostrar los ángulos calculados
disp('Ángulos de Euler ZYZ:');
disp(['alpha (α): ', num2str(alpha)]);
disp(['beta (β): ', num2str(beta)]);
disp(['gamma (γ): ', num2str(gamma)]);

% Función para ajustar un ángulo al rango especificado
function anguloAjustado = ajustarRango(angulo, limiteInferior, limiteSuperior)
    while angulo < limiteInferior
        angulo = angulo + 2*pi;
    end
    while angulo > limiteSuperior
        angulo = angulo - 2*pi;
    end
    anguloAjustado = angulo;
end
