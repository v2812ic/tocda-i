% Inputs: Ninguno 
% Outputs:
%   - Resultados de variables de optimización
%   - Figuras del frente de Pareto
%   - Resumen del proceso
%   - Todo se guarda en Resultados/

clc;
clear;

import Core.evaluarVuelo
addpath('Utils');

%% 0. Control y setup de simulación

aviones = ["A320", "B757", "B737"]; % etc
heuristico = true; % genetico
gradiente = false; 

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

    % Muestra en pantalla de lo que está sucediendo
    fprintf("------------------------------------------------\n");
    fprintf(strcat("Comienza la optimización del avión ", avionActual, "\n\n"));
    % Inicialización del objeto del avión y de las variables de decisión
    try
        avion = Avion(avionActual);
    catch ME
        fprintf(strcat("Deteniendo simulación para avión ", avionActual, ".\n"));
        fprintf(strcat(ME.message, "\n\n"));
        continue; % salta de avión y pasa al siguiente
    end

    if avion.MTOW - avion.OEW - parametrosFijos.PL <= 0
        fprintf('Se aborta la optimización con avión %s porque no puede cargar tanto payload.\n\n', avionActual);
        continue
    end

    % Restricciones lineales de las variables de decision
    lb = [fronterasFijas.x1Min, fronterasFijas.x2Min, fronterasFijas.x3Min, ...
        fronterasFijas.x4Min, avion.vMinDespegue, avion.vMinCrucero, ...
        avion.vMinCrucero, avion.vMinCrucero, avion.vMinAproximacion, ...
        fronterasFijas.hCruceroMin, fronterasFijas.hCruceroMin,0];
    ub = [fronterasFijas.x1Max, fronterasFijas.x2Max, fronterasFijas.x3Max, ...
        fronterasFijas.x4Max, avion.vMaxDespegue, avion.vMaxCrucero, ...
        avion.vMaxCrucero, avion.vMaxCrucero, avion.vMaxAproximacion, ...
        avion.techoDeVuelo, avion.techoDeVuelo, ...
        min(avion.FWmax, avion.MTOW - avion.OEW - parametrosFijos.PL)];

    masterEval = @(X) evaluarVuelo(X, avion, parametrosFijos, fronterasFijas);
    
    fprintf("Carga de datos completada, comienza la optimización.\n")
    %% 2.1. Algoritmo heuristico

    if control.heuristico
    fprintf("Comienza la optimización por algoritmo genético.\n")
        optionsGA = optimoptions('gamultiobj', ...
            'UseParallel',true);

        [X_ga, F_ga, exitflag_ga, output_ga] = gamultiobj(masterEval, ...
            control.nvars, [], [], [], [], lb, ub, optionsGA);

        Resultados.(avionActual).ga.X = X_ga;
        Resultados.(avionActual).ga.F = F_ga;
        Resultados.(avionActual).ga.output = output_ga;

        plotPareto(F_ga, avionActual);
    
    fprintf("Optimización heurística completada.\n\n")  
    end
    
    %% 2.2. Algoritmo de gradiente
    if control.gradiente
    fprintf("Comienza la optimización por algoritmo gradiente.\n")
    fprintf("Optimización gradiente completada.\n\n")  
    end
end
    

%% 3. GUARDADO DE RESULTADOS

fprintf(strcat("\nOptimización completada, guardando resultados.\n\n"));
fechaHora = datetime("now", "Format", "yyyyMMdd_HHmm");
nombreArchivo = fullfile("Resultados", "Resultados_" + char(fechaHora) + ".mat");
save(nombreArchivo, 'Resultados');

