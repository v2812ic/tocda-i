function [objetivos, c, ceq] = evaluarVuelo(X, Avion, parametrosFijos, fronterasFijas)
    import Trayectoria.simularPerfil
    
    fuelInicial = X(8);
    
    % --- 1. SIMULACIÓN ---
    resultados = simularPerfil(X(1:2), X(3:5), X(6:7), fuelInicial, ...
        parametrosFijos, Avion);
    
    % --- 2. OBJETIVOS ---
    objetivos = [resultados.tiempoTotal; resultados.combustibleConsumido/parametrosFijos.PL];
    
    % --- 3. RESTRICCIONES DE IGUALDAD ---
    ceq = resultados.violacionRestricciones;
    
    % --- 4. RESTRICCIONES DE DESIGUALDAD (c <= 0) ---
    % Aumentamos el tamaño de c para incluir los ángulos (7 + 3 = 10)
    c = zeros(7, 1);
    
    % A) COMBUSTIBLE
    c(1) = resultados.combustibleConsumido - fuelInicial / parametrosFijos.seguridadFuel;
    
    % B) CÁLCULOS GEOMÉTRICOS (Deltah y Distancias)
    Deltah = [X(6) - parametrosFijos.h_origen;             % Fase 1: Ascenso inicial
              X(7) - X(6);                                % Fase 2: Cambio nivel
              parametrosFijos.h_destino - X(7)];           % Fase 3: Descenso final

    Distancias = [X(1);                                     % Distancia fase 1
                  X(2);                                     % Distancia fase 2
                  parametrosFijos.distancia - sum(X(1:2))]; % Distancia fase 3

    % Calculamos los ángulos reales de las fases de transición
    gammas = atan(Deltah ./ Distancias);


    c(2) = gammas(1) - fronterasFijas.maxTasaAscenso;
    c(3) = fronterasFijas.minTasaAscenso - gammas(1);

    c(4) = gammas(2) - fronterasFijas.maxTasaAscenso;
    c(5) = fronterasFijas.maxTasaDescenso - gammas(2);

    c(6) = gammas(3) - fronterasFijas.minTasaDescenso;
    c(7) = fronterasFijas.maxTasaDescenso - gammas(3);

end