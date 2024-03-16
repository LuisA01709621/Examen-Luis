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
