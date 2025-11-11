% Función del optimizador. Es el constructor del problema de optimización.
%
% Inputs:
%   X           (vector 1x12) Vector de variables de decisión
%   fixedParams (struct)  Estructura con parámetros fijos (payload, dist, etc.)
%   avionObj    (objeto)  Objeto de la clase 'Avion'
%
% Outputs:
%   objetivos   (vector 2x1) [TiempoTotal; ConsumoNormalizado]
%   c           (vector Nx1) Restricciones de DESIGUALDAD (c <= 0)
%   ceq         (vector Mx1) Restricciones de IGUALDAD (ceq = 0)

function [objetivos, c, ceq] = evaluarVuelo(X, fixedParams, avionObj)

    %% 1. DESEMPAQUETAR EL VECTOR DE DECISIÓN (X)
    dist_fases_4   = X(1:4); 
    vel_fases_5    = X(5:9);   
    alt_cruceros_2 = X(10:11);
    fuel_inicial   = X(12);   

    %% 2. LLAMAR AL SIMULADOR 
    try
        resultados = simularPerfil(dist_fases_4, vel_fases_5, alt_cruceros_2, ...
                                   fuel_inicial, fixedParams, avionObj);
        
        simFailed = resultados.simFailedFlag; 
        
    catch ME
        fprintf('¡Simulación fallida! Error: %s\n', ME.message);
        simFailed = true;
    end

    %% 3. GESTIÓN DE SIMULACIÓN FALLIDA
    if simFailed
        % Penalización ENORME si la simulación falla
        objetivos = [1e10; 1e10]; 
        c         = 1e10;        
        ceq       = 1e10;        
        return; 
    end

    %% 4. CALCULAR OBJETIVOS
    obj1 = resultados.tiempoTotal; 
    
    if fixedParams.payload > 0
        obj2 = resultados.combustibleConsumido / fixedParams.payload; 
    else
        obj2 = resultados.combustibleConsumido; 
    end
    
    objetivos = [obj1; obj2];

    %% 5. CALCULAR RESTRICCIONES
    
    % --- Restricciones de IGUALDAD (ceq = 0) ---
    ceq = [];
    % La distancia total recorrida debe ser la distancia objetivo
    ceq(1) = resultados.distanciaTotalRecorrida - fixedParams.distanciaTotal;

    
    % --- Restricciones de DESIGUALDAD (c <= 0) ---
    c = [];
    
    % A. Restricciones de Masa
    % (Asumir que Avion.OEW existe en el objeto)
    masa_inicial = fuel_inicial + fixedParams.payload + avionObj.OEW;
    c(end+1) = masa_inicial - avionObj.MTOW; [cite_start]% (masa_ini <= MTOW) [cite: 25]

    % B. Reserva de Combustible
    c(end+1) = fixedParams.reservaMinima - resultados.combustibleRestante; [cite_start]% (reservaMin <= restante) [cite: 26]
    
    % C. Límites de Vuelo (Altitud y Velocidad)
    c(end+1) = alt_cruceros_2(1) - avionObj.h_max; [cite_start]% (h_crucero1 <= h_max) [cite: 28]
    c(end+1) = avionObj.h_min - alt_cruceros_2(1); [cite_start]% (h_min <= h_crucero1) [cite: 28]
    c(end+1) = alt_cruceros_2(2) - avionObj.h_max; [cite_start]% [cite: 28]
    c(end+1) = avionObj.h_min - alt_cruceros_2(2); [cite_start]% [cite: 28]

    c = [c; vel_fases_5' - avionObj.V_max]; [cite_start]% (V_fase <= V_max) [cite: 23]
    c = [c; avionObj.V_min - vel_fases_5']; [cite_start]% (V_min <= V_fase) [cite: 23]

    % D. Restricciones de Simulación
    % (Asumir que 'simularPerfil' las calcula y devuelve)
    [cite_start]% c(end+1) = resultados.maxClimbRate - fixedParams.maxClimbRateAllowed; [cite: 24]
    
    % E. Restricción Lógica de Distancias
    c(end+1) = sum(dist_fases_4) - fixedParams.distanciaTotal;
    
    % Asegurar que son vectores columna
    c = c(:);
    ceq = ceq(:);
    
end