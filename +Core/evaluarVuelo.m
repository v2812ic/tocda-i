% Función del optimizador
% 
% Inputs:
%   X           (vector 1x12) Vector de variables de decisión del optimizador
%               - X(1:4): Distancias de las fases [dist1, dist2, dist3, dist4] (m)
%               - X(5:9): Velocidades de las fases [V1, V2, V3, V4, V5] (m/s)
%               - X(10:11): Alturas de crucero [h_crucero1, h_crucero2] (m)
%               - X(12): Cantidad de combustible cargada (kg)
%
%   fixedParams (struct)  Estructura con parámetros fijos del problema.
%               - .payload (kg) 
%               - .h_salida, .h_llegada (m) 
%               - .distanciaTotal (m)
%               - .reservaMinima (kg)
%
%   avionObj    (objeto)  Objeto de la clase 'Avion' que contiene todos los
%                         datos y métodos de la aeronave (Aero y Motor).
%
% Outputs:
%   objetivos   (vector 1x2) Vector de los dos objetivos a minimizar.
%               - objetivos(1): Tiempo total de vuelo (s) 
%               - objetivos(2): Consumo combustible normalizado (kg/kg)
%
%   c           (vector Nx1) Vector de restricciones de DESIGUALDAD (c(x) <= 0).
%               - ej. (masaInicial - avionObj.MTOW)
%               - ej. (fixedParams.reservaMinima - combustibleRestante)
%               - ej. (velocidadSimulada - avionObj.Vmax)
%
%   ceq         (vector Mx1) Vector de restricciones de IGUALDAD (ceq(x) = 0).
%               - ej. (distanciaTotalRecorrida - fixedParams.distanciaTotal)