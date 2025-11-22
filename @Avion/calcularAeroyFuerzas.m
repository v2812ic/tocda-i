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
%h viene de la función calcular perfil
%v viene de la función calcular perfil
%W viene de la función calcular perfil
if h<=11000
    rho=1.225*(1-0.000022558*h)^4.2559
else
    rho=0.03639*exp(-0.00015769*(h-11000))
end
Fy=-W %inicializamos las fuerzas verticales
gamma=0 %inicializamos gamma
%Mientras Fy no supere cero, incrementamos el ángulo de asiento en una centésima, lo cual incrementará a su vez la sustentación generada por el avión. Dado que un gamma=0 da un cL de 0,5, podemos inicializar gamma en un valor inferior a cero.
while Fy<0
    Fy=(1/2)*rho*v^2*S*(0.1*gamma+0.5)-W*cos(deg2rad(gamma))
    if Fy>=0
        gamma=gamma
    else
        gamma=gamma+0.01
    end
end


if gamma==10
    while Fy<0
        Fy=(1/2)*rho*v^2*S*(-(0.009*gamma^2 )+0.298*gamma-0.59)-W*cos(deg2rad(gamma))
        if Fy>=0
            gamma=gamma
        else
            gamma=gamma+0.01
        end
    end
end

%comprobamos si la centésima que da el valor que queda por encima está más cerca de cero que la anterior, para quedarnos con aquella que dé menor diferencia
Fy_debajo=(1/2)*rho*v^2*S*(0.1*(gamma-0.01)+0.5)-W*cos(deg2rad(gamma-0.01))
if abs(Fy_debajo)<Fy
    gamma=gamma-0.01
end
%Calculamos el cD
c_D=0.00036*gamma^2+0.00036*gamma+0.029
%Calculamos D
D=(1/2)*rho*v^2*S*c_D
%Sumamos a D la componente del peso en el eje horizontal del avión. Importante llamarlo empuje porque es el nombre que hemos dado al principio de la función
empuje=D+W*sin(deg2rad(gamma))
end
