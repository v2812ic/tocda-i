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
