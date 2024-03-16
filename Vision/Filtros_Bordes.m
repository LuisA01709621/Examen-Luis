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
