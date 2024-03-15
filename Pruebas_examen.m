%%
clc
close all

a=1;
b=1;
c=-8;
hold on
grid on
plot(out.x2,out.y2,'LineWidth',2);
hold on
plot(10,-2,'o','LineWidth',2)
hold on
plot(4,4,'o','LineWidth',2)
title('La trayectoria del robot')
legend('Trayectoria CLR')
xlabel('X')
ylabel('Y')
%%
clc
close all
sigma = 40;                                 % Define el valor de la desviación estándar del ruido.
im = imread('Castillo.png');                % Lee la imagen 'descarga.jpg' del directorio actual.

m2 = [1 2 1;                                % Define una máscara para un filtro de paso bajo ponderado.
      2 4 2;
      1 2 1];
f2_im = imfilter(im, m2/8, 'corr');    % Aplica el filtro ponderado a la imagen con ruido.
figure;
imshow(f2_im);
fa_im = imfilter(im, m2/11, 'corr');    % Aplica el filtro ponderado a la imagen con ruido.
figure;
imshow(fa_im);


im_gray = rgb2gray(f2_im);
figure;
imshow(im_gray);
im_gray2 = rgb2gray(fa_im);
figure;
imshow(im_gray2);
m8 = [1 2 1;
      0 0 0;
      -1 -2 -1];
m9 = [-1 0 1;
      -2 0 2;
      -1 0 1];
f7_im = imfilter(im_gray, m9, 'conv');    % Aplica el filtro ponderado a la imagen con ruido.
figure;
imshow(f7_im);
f8_im = imfilter(im_gray2, m8, 'conv');    % Aplica el filtro ponderado a la imagen con ruido.
figure;
imshow(f8_im);
%%
clc,clear
syms q1 q2 d3 l1 l2

parametrosDH=[q1 l1 0 pi/2;pi/2+q2 0 -l2 pi/2;0 d3 0 0];

% Obtener el número de filas (n) en la tabla de parámetros DH
n = size(parametrosDH, 1);

% Inicializar la matriz homogénea como la matriz identidad 4x4
matrizHomogenea = eye(4);

% Inicializar el arreglo para almacenar las transformaciones homogéneas
transformaciones = cell(1, n);

% Crear un arreglo para almacenar cada transformación homogénea en una variable
for i = 1:n
    eval(sprintf('T%d%d = eye(4);', i-1, i)); % Crear una variable con el nombre Tij
end

% Iterar sobre cada fila de la tabla de parámetros DH
for i = 1:n
    % Extraer los parámetros a, d, a1, y phi_i de la fila actual
    ai = parametrosDH(i, 1);
    di = parametrosDH(i, 2);
    ai1 = parametrosDH(i, 3);
    phi_i = parametrosDH(i, 4);

    % Construir la matriz de transformación homogénea Ai para la i-ésima articulación
    Ai = [
        cos(ai), -sin(ai), 0, di;
        sin(ai)*cos(phi_i), cos(ai)*cos(phi_i), -sin(phi_i), -sin(phi_i)*ai1;
        sin(ai)*sin(phi_i), cos(ai)*sin(phi_i), cos(phi_i), cos(phi_i)*ai1;
        0, 0, 0, 1
    ];
    
    % Almacenar la transformación homogénea en el arreglo y en variables individuales
    transformaciones{i} = Ai;
    eval(sprintf('T%d%d = Ai;', i-1, i)); % Asignar la matriz a la variable correspondiente

    % Imprimir la transformación homogénea
    fprintf('T%d%d = \n', i-1, i);
    disp(Ai);

    % Actualizar la matriz homogénea acumulativa multiplicándola por Ai
    matrizHomogenea = matrizHomogenea * Ai;
end

disp('T02:')
T02 = T01*T12;
disp(T02)
disp('T03:')
T03 = T01*T12*T23;
disp(T03)
disp('T13:')
T13 = T12*T23;
disp(T13)
%% Matriz de Rotación a RPY
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

%% Matriz de rotacion a ZYZ
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