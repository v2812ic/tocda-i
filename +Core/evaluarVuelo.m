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

% Imports de función de simulación, tiene que estar dentro de la función
% para que la función sea reconocida
import Trayectoria.simularPerfil

    distFases = [X(1:4)];
    velFases = [X(5:9)];
    altitudCruceros = [X(10:11)];
    fuelInicial = X(12);
    
    resultados = simularPerfil(distFases, velFases, altitudCruceros, fuelInicial, ...
        parametrosFijos, Avion);
    
    objetivos = [resultados.tiempoTotal, resultados.combustibleConsumido];
    
    % restricciones de igualdad (que no se rompa la física solamente)
    ceq = resultados.violacionRestricciones;
    
    % restricciones de desigualdad (que llegue a destino sin romper la física y
    % que no gaste todo el combustible)
    c(1) = resultados.combustibleConsumido - parametrosFijos.seguridadFuel * fuelInicial;
    c(2) = -sum(distFases) + parametrosFijos.distancia - fronterasFijas.x5Max;
    c(3) = sum(distFases) - parametrosFijos.distancia + fronterasFijas.x5Min;

end