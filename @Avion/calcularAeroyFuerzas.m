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

%function [L, D, T] = calcularAeroyFuerzas(avion)

% ME ESTOY RAYANDO CON CÓMO IMPLEMENTAR EL CÁLCULO DE alpha. Revisar que
% creo que está mal configurada esta función con respecto a lo que se
% declara en la clase

%L = pi;
%cL = pi;

% distinguir entre fases según el estado
%D = avion.cd0 + avion.k*(cL^2);

%end



function empuje=calcular_fuerzas_aero(v,h,W)
%S viene de los datos del avión
%h viene de la función simular perfil
%v viene de la función simular perfil
%W viene de la función simular perfil
%gamma viene de la función simular perfil
%Cdo viene de los datos del avión
%k viene de los datos del avión
if h<=11000
    rho=1.225*(1-0.000022558*h)^4.2559;
else
    rho=0.03639*exp(-0.00015769*(h-11000));
end

Cl=W*cos(deg2rad(gamma))/((1/2)*rho*v^2*S);
    
%Calculamos el Cd
Cd=k*Cl+Cdo;
%Calculamos D
D=(1/2)*rho*v^2*S*Cd;
%Sumamos a D la componente del peso en el eje horizontal del avión. Importante llamarlo empuje porque es el nombre que hemos dado al principio de la función
empuje=D+W*sin(deg2rad(gamma));
