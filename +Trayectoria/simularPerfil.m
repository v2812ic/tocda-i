% Inputs:
%   (variables desempaquetadas de evaluarVuelo)
%   dist_fases   (vector 1x4) Distancias horizontales de las fases 1-4 (m)
%   vel_fases    (vector 1x5) Velocidades CAS/Mach para cada fase (m/s o Mach)
%   alt_cruceros (vector 1x2) Altitudes de crucero 1 y 2 (m)
%   fuel_inicial (scalar)    Masa de combustible inicial (kg)
%   fixedParams  (struct)     Ver Core.evaluarVuelo
%   avionObj     (objeto)     Objeto de la clase 'Avion'
%   handles de funciones
%
% Outputs:
%   resultados   (struct) Estructura con los resultados de la simulación.
%               - .tiempoTotal (s)
%               - .combustibleConsumido (kg)
%               - .combustibleRestante (kg)
%               - .distanciaTotalRecorrida (m)
%               - .violacionRestricciones (scalar) Penalización si fsolve
%               falla (flag de error)
%