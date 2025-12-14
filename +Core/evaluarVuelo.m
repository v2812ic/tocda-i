% Función del optimizador. Es el constructor del problema de optimización.
%
% Inputs:
%   X           (vector 1x12) Vector de variables de decisión
%   fixedParams (struct)  Estructura con parámetros fijos (payload, dist, etc.)
%   avionObj    (objeto)  Objeto de la clase 'Avion'
%
% Outputs:
%   objetivos   (vector 2x1) [TiempoTotal; ConsumoNormalizado]
%   c           (vector Nx1) Restricciones de DESIGUALDAD (c <= 0) (no
%   lineales, no hay)
%   ceq         (vector Mx1) Restricciones de IGUALDAD (ceq = 0) (no
%   lineales, solo hay la del combustible)

%% Función de construcción del problema no lineal

function [objetivos, c, ceq] = evaluarVuelo(X, Avion, parametrosFijos, fronterasFijas)
    import Trayectoria.simularPerfil

    % Desempaquetado (claridad)
    distAscenso = X(1);
    distDescenso = X(4);
    velAscenso = X(5);
    velDescenso = X(9);
    hCrucero = X(10); 
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
    
    % A) Combustible suficiente
    c(idx) = resultados.combustibleConsumido - parametrosFijos.seguridadFuel * fuelInicial;
    idx = idx + 1;
    
    % B) Distancia Total (Mínima y Máxima requerida)
    distanciaCalculada = sum(X(1:4));
    % Que no sea más corta que la ruta
    c(idx) = (parametrosFijos.distancia - fronterasFijas.x5Min) - distanciaCalculada;
    idx = idx + 1;
    % Que no sea más larga que la ruta (si aplica)
    c(idx) = distanciaCalculada - (parametrosFijos.distancia + fronterasFijas.x5Max); 
    idx = idx + 1;

    % C) FÍSICA DE ASCENSO (Evita subidas verticales)
    % Delta Altura Ascenso
    deltaH_asc = hCrucero - parametrosFijos.h_origen;
    
    % Tasa Max: deltaH * v - MaxRoC * dist <= 0
    c(idx) = (deltaH_asc * velAscenso) - (fronterasFijas.maxTasaAscenso * distAscenso);
    idx = idx + 1;
    
    % Tasa Min: MinRoC * dist - deltaH * v <= 0
    c(idx) = (fronterasFijas.minTasaAscenso * distAscenso) - (deltaH_asc * velAscenso);
    idx = idx + 1;
    
    % D) FÍSICA DE DESCENSO
    % Delta Altura Descenso
    deltaH_des = hCrucero - parametrosFijos.h_destino;
    
    % Tasa Max Descenso
    c(idx) = (deltaH_des * velDescenso) - (fronterasFijas.maxTasaDescenso * distDescenso);
    idx = idx + 1;
    
    % Tasa Min Descenso
    c(idx) = (fronterasFijas.minTasaDescenso * distDescenso) - (deltaH_des * velDescenso);
end