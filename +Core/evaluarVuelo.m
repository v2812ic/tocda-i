% EVALUARVUELO Evalúa la función de coste y restricciones para la optimización de trayectoria.
%
%   Esta función actúa como el constructor del problema de optimización no lineal.
%   Recibe el vector de decisión X, simula el vuelo llamando a 'simularPerfil',
%   y construye los vectores de objetivos y restricciones.
%
%   ESTADO: VERIFICADO Y FUNCIONAL (Test unitario superado).
%
%   SINTAXIS:
%       [objetivos, c, ceq] = evaluarVuelo(X, Avion, parametrosFijos, fronterasFijas)
%
%   ENTRADAS:
%       X               (1x12 double) Vector de variables de decisión:
%                           X(1:4): Distancias horizontales de fases (m)
%                           X(5:9): Velocidades TAS (m/s)
%                           X(10:11): Altitudes clave (m)
%                           X(12): Combustible inicial (kg)
%       Avion           (Objeto) Instancia de la clase Avion con modelo de performance.
%       parametrosFijos (struct) Parámetros de la misión (distancia total, h_origen, etc.).
%       fronterasFijas  (struct) Límites físicos y operativos:
%                           .x5Min, .x5Max: Límites geometría descenso.
%                           .maxTasaAscenso: Límite positivo (ej. +20 m/s).
%                           .maxTasaDescenso: Límite NEGATIVO (ej. -15 m/s).
%
%   SALIDAS:
%       objetivos       (2x1 double) [TiempoTotal (s); Consumo (kg)].
%       c               (Nx1 double) Restricciones de DESIGUALDAD (c <= 0).
%       ceq             (Mx1 double) Restricciones de IGUALDAD (ceq = 0).
%
%   MAPEO DE RESTRICCIONES DE DESIGUALDAD (c):
%       c(1)   : Suficiencia de combustible (con factor de seguridad).
%       c(2)   : Distancia mínima para descenso (Geometría).
%       c(3)   : Distancia máxima para descenso (Geometría).
%       c(4-5) : Límites Tasa Ascenso/Descenso - Fase 1 (Ascenso).
%       c(6-7) : Límites Tasa Ascenso/Descenso - Fase 3 (Ajuste).
%       c(8-9) : Límites Tasa Ascenso/Descenso - Fase 5 (Descenso).
%
%   Ver también TRAYECTORIA.SIMULARPERFIL
% 

function [objetivos, c, ceq] = evaluarVuelo(X, Avion, parametrosFijos, fronterasFijas)

import Trayectoria.simularPerfil
    
    fuelInicial = X(12);
    
    % --- 1. SIMULACIÓN ---
    resultados = simularPerfil(X(1:4), X(5:9), X(10:11), fuelInicial, ...
        parametrosFijos, fronterasFijas, Avion);
    
    % --- 2. OBJETIVOS ---
    objetivos = [resultados.tiempoTotal; resultados.combustibleConsumido];
    
    % --- 3. RESTRICCIONES DE IGUALDAD (ceq = 0) ---
    ceq = [resultados.violacionRestricciones]; 
    
    % --- 4. RESTRICCIONES DE DESIGUALDAD (c <= 0) ---
    c = [];
    idx = 1;
    
    % -----------------------------------------------------
    % A) COMBUSTIBLE
    % -----------------------------------------------------
    % c(1): El combustible consumido supera al inicial (con margen de seguridad)
    c(idx) = resultados.combustibleConsumido - fuelInicial / parametrosFijos.seguridadFuel;
    idx = idx + 1;
    
    % -----------------------------------------------------
    % B) GEOMETRÍA DE LA RUTA
    % -----------------------------------------------------
    distanciaCalculada = sum(X(1:4));
    
    % c(2): La ruta calculada es demasiado corta (deja demasiado espacio para el descenso)
    % Matemáticamente: DistanciaCalculada < (Total - MaxDescenso)
    c(idx) = (parametrosFijos.distancia - fronterasFijas.x5Max) - distanciaCalculada;
    idx = idx + 1;
    
    % c(3): La ruta calculada es demasiado larga (no deja espacio para bajar)
    % Matemáticamente: DistanciaCalculada > (Total - MinDescenso)
    c(idx) = distanciaCalculada - (parametrosFijos.distancia - fronterasFijas.x5Min); 
    idx = idx + 1;
    
    % -----------------------------------------------------
    % C) FÍSICAS (Velocidades Verticales)
    % -----------------------------------------------------
    % Definición de Deltas de Altura
    Deltah1 = X(10) - parametrosFijos.h_origen;             % Altura ganada fase 1
    Deltah2 = X(11) - X(10);                                % Altura ganada/perdida fase 3 (escalón)
    Deltah3 = parametrosFijos.h_destino - X(11);            % Altura perdida fase 5 (descenso)
    
    % Cálculo trigonométrico exacto de Velocidad Vertical (Vv = Vtas * sin(gamma))
    % Nota: X(1)=Dist1, X(3)=Dist3 (Ajuste), DistDescenso = Total - calculada
    Vv1 = X(5) * sin(atan(Deltah1 / X(1))); 
    Vv2 = X(7) * sin(atan(Deltah2 / X(3))); 
    Vv3 = X(9) * sin(atan(Deltah3 / (parametrosFijos.distancia - distanciaCalculada))); 
    
    velocidadesVerticales = [Vv1, Vv2, Vv3];
    
    % BUCLE DE RESTRICCIONES FÍSICAS
    % k=1 -> Ascenso Inicial
    % k=2 -> Ajuste (Escalón)
    % k=3 -> Descenso Final
    
    for k = 1:3
        vActual = velocidadesVerticales(k);
        
        % Restricción de TECHO (Máximo Ascenso)
        % Índices resultantes: c(4) para Fase1, c(6) para Fase3, c(8) para Fase5
        c(idx) = vActual - fronterasFijas.maxTasaAscenso;
        idx = idx + 1;
        
        % Restricción de SUELO (Máximo Descenso / Mínimo Ascenso negativo)
        % IMPORTANTE: fronterasFijas.maxTasaDescenso debe ser NEGATIVO (ej. -15)
        % Índices resultantes: c(5) para Fase1, c(7) para Fase3, c(9) para Fase5
        c(idx) = fronterasFijas.maxTasaDescenso - vActual;
        idx = idx + 1;
    end
end