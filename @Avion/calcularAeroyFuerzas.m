% Inputs:
%   obj         (objeto) La instancia de la clase 'Avion' (inyectado)
%   V           (scalar) Velocidad actual (m/s)
%   h           (scalar) Altitud actual (m)
%   alpha       (scalar) Ángulo de ataque (rad)
%   rho         (scalar) Densidad del aire actual (kg/m^3)
%
% Outputs:
%   L           (scalar) Fuerza de sustentación (N)
%   D           (scalar) Fuerza de resistencia (N)

function [L, D, T] = calcularAeroyFuerzas(avion)

% ME ESTOY RAYANDO CON CÓMO IMPLEMENTAR EL CÁLCULO DE alpha. Revisar que
% creo que está mal configurada esta función con respecto a lo que se
% declara en la clase

L = pi;
cL = pi;

% distinguir entre fases según el estado
D = avion.cd0 + avion.k*(cL^2);
end