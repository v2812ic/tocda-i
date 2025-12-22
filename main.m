% Inputs: Ninguno 
% Outputs:
%   - Resultados de variables de optimización
%   - Figuras del frente de Pareto
%   - Resumen del proceso
%   - Todo se guarda en Resultados/
%%
% CONSTRUCCION DEL AVION Y FUNCIONAMIENTO DEL MAIN OKEY. COMPROBADO 100% 
%%
% 
% 

clc;
clear;
close all;

import Core.evaluarVuelo
%import Core.evaluarVueloTest
addpath('Utils');

%% 0. Control y setup de simulación


aviones = ["BC300"]; % etc
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
    lb = [min(fronterasFijas.x1Min, parametrosFijos.distancia), min(fronterasFijas.x2Min, parametrosFijos.distancia), min(fronterasFijas.x3Min, parametrosFijos.distancia), ...
        min(fronterasFijas.x4Min, parametrosFijos.distancia), avion.vMinDespegue, avion.vMinCrucero, ...
        avion.vMinCrucero, avion.vMinCrucero, avion.vMinAproximacion, ...
        fronterasFijas.hCruceroMin, fronterasFijas.hCruceroMin,0];
    ub = [min(fronterasFijas.x1Max, parametrosFijos.distancia), min(fronterasFijas.x1Max, parametrosFijos.distancia), min(fronterasFijas.x1Max, parametrosFijos.distancia), ...
        min(fronterasFijas.x1Max, parametrosFijos.distancia), avion.vMaxDespegue, avion.vMaxCrucero, ...
        avion.vMaxCrucero, avion.vMaxCrucero, avion.vMaxAproximacion, ...
        avion.techoDeVuelo, avion.techoDeVuelo, ...
        min(avion.FWmax, avion.MTOW - avion.OEW - parametrosFijos.PL)];

    masterEval = @(X) evaluarVuelo(X, avion, parametrosFijos, fronterasFijas);
    % masterEval = @(X) evaluarVueloTest(X, avion, parametrosFijos, fronterasFijas);
    % Activar linea 77 para test. Ha pasado ok
    objFun = @(x) getOutput(masterEval, x); 
    nonlconFun = @(x) getConstraints(masterEval, x);
    
    fprintf("Carga de datos completada, comienza la optimización.\n")

    %%

    model = 'Turbofan_Model';
    load_system(model);
    %set_param(model,'SimulationCommand','update');
    set_param(model,'FastRestart','off');

    %% 2.1. Algoritmo heuristico

    if control.heuristico
        fprintf("Comienza la optimización por algoritmo genético.\n")
        optionsGA = optimoptions('gamultiobj', ...
            'UseParallel', true, ...
            'Display', 'iter');
        
        [X_ga, F_ga, exitflag_ga, output_ga] = gamultiobj(objFun, ...
            control.nvars, [], [], [], [], lb, ub, nonlconFun, optionsGA);
            
        Resultados.(avionActual).ga.X = X_ga;
        Resultados.(avionActual).ga.F = F_ga;
        Resultados.(avionActual).ga.output = output_ga;
        
        if exist('plotPareto','file'), plotPareto(F_ga, avionActual); end
        fprintf("   Optimización heurística completada.\n\n")  
    end
    
    %% 2.2. Algoritmo de gradiente
    if control.gradiente

        fprintf("Comienza la optimización por algoritmo basado en gradiente.\n");
        
        x0 = (lb + ub) / 2;

        funcionCosteEscalar = @(x) sumaPonderada(x, masterEval, w1, w2);
        
        optionsGrad = optimoptions('fmincon', ...
            'Display', 'iter', ...
            'Algorithm', 'sqp','UseParallel',true); 
            
        [X_grad, J_val, exitflag_grad, output_grad] = fmincon(funcionCosteEscalar, ...
            x0, [], [], [], [], lb, ub, [], optionsGrad);
            
        F_grad = masterEval(X_grad);
        
        Resultados.(avionActual).grad.X = X_grad;
        Resultados.(avionActual).grad.F = F_grad; 
        Resultados.(avionActual).grad.J = J_val;
        Resultados.(avionActual).grad.output = output_grad;
        
        fprintf("   Optimización gradiente completada.\n\n");
    end
end
    

%% 3. GUARDADO DE RESULTADOS

fprintf(strcat("\nOptimización completada, guardando resultados.\n\n"));
fechaHora = datetime("now", "Format", "yyyyMMdd_HHmm");
nombreArchivo = fullfile("Resultados", "Resultados_" + char(fechaHora) + ".mat");
save(nombreArchivo, 'Resultados');

%% 3. VISUALIZACIÓN DE DATOS EN CONSOLA
    fprintf('\n======================================================\n');
    fprintf(' RESUMEN DE RESULTADOS PARA: %s\n', avionActual);
    fprintf('======================================================\n');
    
    if control.gradiente
        fprintf('\n>>> RESULTADOS GRADIENTE (fmincon) <<<\n');
        fprintf('  Exit Flag: %d (1 = Convergencia Exitosa)\n', exitflag_grad);
        fprintf('  Iteraciones: %d\n', output_grad.iterations);
        fprintf('------------------------------------------------------\n');
        fprintf('  COSTO TOTAL (J = w1*f1 + w2*f2): %.6f\n', J_val);
        fprintf('  OBJETIVOS [f1, f2]:              [%.4f,  %.4f]\n', F_grad(1), F_grad(2));
        fprintf('------------------------------------------------------\n');
        fprintf('  VARIABLES DE DISEÑO (X):\n');
        % Imprimimos las 12 variables en dos filas para que se lea bien
        fprintf('    x1-x6:  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', X_grad(1:6));
        fprintf('    x7-x12: %.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', X_grad(7:12));
        
        % INTERPRETACIÓN PARA EL TEST ZDT1
        if exist('MODO_TEST', 'var') && MODO_TEST
            fprintf('\n  [ANÁLISIS TEST ZDT1]:\n');
            fprintf('  Con w1=0.5 y w2=0.5, el óptimo teórico debe equilibrar f1 y f2.\n');
            fprintf('  Si f1 está entre 0.20 y 0.30, el gradiente ha funcionado PERFECTO.\n');
        end
    end

    if control.heuristico
        fprintf('\n>>> RESULTADOS GENÉTICO (GAMULTIOBJ) <<<\n');
        fprintf('  Puntos encontrados en Pareto: %d\n', size(F_ga, 1));
        fprintf('  Rango f1 (min - max): %.4f - %.4f\n', min(F_ga(:,1)), max(F_ga(:,1)));
        fprintf('  Rango f2 (min - max): %.4f - %.4f\n', min(F_ga(:,2)), max(F_ga(:,2)));
        fprintf('  (Ver gráfica para distribución visual)\n');
    end
    fprintf('\n======================================================\n\n');



%% 4. FUNCIONES AUXILIARES
function f = getOutput(funHandle, x)
    % Llama a evaluarVuelo y se queda solo con el primer output (objetivos)
    [f, ~, ~] = funHandle(x);
end

function [c, ceq] = getConstraints(funHandle, x)
    % Llama a evaluarVuelo y se queda con el segundo y tercero (restricciones)
    [~, c, ceq] = funHandle(x);
end

function J = sumaPonderada(x, funHandle, w1, w2)
    % 1. Obtenemos el vector de objetivos [f1, f2] evaluando la función maestra
    [f, ~, ~] = funHandle(x);
    
    % 2. Calculamos el escalar (suma ponderada)
    % Importante: f(1) es tiempo (o equivalente ZDT1), f(2) es consumo
    J = w1 * f(1) + w2 * f(2);
end