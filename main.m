% Inputs: Ninguno 
% Outputs:
%   - Resultados de variables de optimización
%   - Figuras del frente de Pareto
%   - Resumen del proceso
%   - Todo se guarda en Resultados/

clc;
clear;

%% 0. Control y setup de simulación

aviones = ["A320", "B737", "B757"]; % etc
heuristico = true; % genetico
gradiente = true; 

w1 = 0.5; % 50% importancia al tiempo
w2 = 0.5; % 50% importancia al consumo

nvars = 12;

%% 1. Carga de datos del problema

% Construye el controlador de opciones de la simulación
control.aviones = aviones;
control.heuristico = heuristico;
control.gradiente = gradiente;
control.w1 = w1;
control.w2 = w2;
control.nvars = nvars;

% Carga las restricciones generales (distancias de aeropuertos) y las
% fronteras de las restricciones (máximos y mínimos)
restriccionesGenerales = load("Data/restriccionesGenerales.mat");
parametrosFijos = restriccionesGenerales.parametros;
fronterasFijas = restriccionesGenerales.fronteras;

for i = 1:length(aviones)

    avionActual = aviones(i);
    
    % Inicialización del objeto del avión y de las variables de decisión
    Avion = Avion(avionActual);

    %% 2.1. Algoritmo heuristico

    if control.heuristico

    end
    
    %% 2.2. Algoritmo de gradiente
    if control.gradiente

    end
end
    

%% 3. GUARDADO DE RESULTADOS
save(['Resultados/Resultados_' datetime("now",'yyyymmdd_HHMM') '.mat'], 'Resultados');

