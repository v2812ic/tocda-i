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


function [empuje, alpha, exitflag] = calcularAeroyFuerzas(v, h, W, gamma, avion, fase)
% Calcula empuje y ángulo de ataque resolviendo el equilibrio aerodinámico
%
% Outputs:
% empuje : Empuje requerido (N)
% alpha  : Ángulo de ataque (rad)
% exitflag : Estado de convergencia de fsolve

% Esto se debería meter en la clase del avión, de todas maneras

tic;

    %% Selección de CD0 según fase
    if isnan(v) || isnan(h) || isnan(W)
        % Si los datos de entrada son basura, devolvemos valores de error
        empuje = NaN; alpha = NaN; exitflag = -2;
        return;
    end
    
    switch fase
        case 1
            CD0 = avion.cD0Despegue;
        case {2,3,4}
            CD0 = avion.cD0Crucero;
        case 5
            CD0 = avion.cD0Descenso;
        otherwise
            warning('Fase no reconocida, usando CD0 de crucero para no explotar, pero para ');
            CD0 = avion.cD0Crucero;
    end

    %% Parámetros del avión
    k   = avion.k;
    S   = avion.S;
    Cla = avion.Cla;  

    %% Atmósfera
    if h <= 11000
        rho = 1.225*(1 - 0.000022558*h)^4.2559;
    else
        rho = 0.3639*exp(-0.00015769*(h-11000));
    end

    %% Resolver sistema no lineal
    x0 = [deg2rad(0); W/10];   % [alpha0; T0]

    options = optimoptions('fsolve', ...
        'Display','off', ...
        'FunctionTolerance',1e-10);

    [x,~,exitflag] = fsolve(@equations, x0, options);
    
    alpha  = x(1);
    empuje = x(2);

    %% Sistema de ecuaciones
    function F = equations(x)
        alpha = x(1);
        T     = x(2);

        CL = Cla * alpha;
        CD = CD0 + k * CL^2;

        L = 0.5 * rho * v^2 * S * CL;
        D = 0.5 * rho * v^2 * S * CD;

        F(1) = T - D - W * sin(gamma);
        F(2) = W * cos(gamma) - L - T * alpha;
    end

tiempo = toc;
%fprintf('Tiempo de ejecución calcular Aero: %.4f segundos\n', tiempo);
end

