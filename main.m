% Inputs: Ninguno 
% Outputs:
%   - Resultados de variables de optimización
%   - Figuras del frente de Pareto
%   - Resumen del proceso
%   - Todo se guarda en Resultados/


% Definición: 
% Inicialización de variables y de la clase del avión, llamada a la función de gamulobj (para
% optimización heurística) y a fmincon (para optimización gradiente). Esto
% habrá que configurarlo para que se llamen a los distintos tipos de
% algoritmos que hayan. De todas formas, al algoritmo solo le interesa el
% problema de optimización que es el que se ha de construir en evaluarVuelo

clc;
clear;

aviones = ["a320", "b737", "b757"]; % etc
datosGenerales = load("Data/restriccionesGenerales.mat");

for i = 1:length(aviones)
    avionActual = aviones(i);
    
    % Inicialización del objeto del avión
    Avion = Avion(avionActual);

    % Llamar al optimizador
end