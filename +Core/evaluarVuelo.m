% Función del optimizador. Es el constructor del problema de optimización y
% se le pasa a la función de optimización (gamulobj o fmincon o quien sea)
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

% La función objetivo es f(X) = penalización por tiempo + penalización por
% consumo de combustible.
% La penalización por tiempo es el tiempo total de vuelo que es
% directamente la suma de dist_i/(V_i * cos(gamma_i), siendo gamma_i
% función de las alturas de vuelo y de las distancias
% La penalización por consumo es la integral de la derivada del gasto
% másico de combustible. Este gasto másico vendrá en última instancia del
% calculaMotor, se integrará en simularPerfil.m. 
% 
% Se puede ver que f(X) = f1(X) + f2(X) tal que f1(X) es el tiempo de vuelo y
% f2(X) es el gasto de combustible. Aparte, todas las restricciones son de
% desigualdad (mirando la lista de restricciones del overleaf) y se puede
% ver que todas son lineales excepto una, la de la reserva. Concretamente,
% esta restricción es: 
% aFW - f2(X) >= b. Siendo f2(X) exactamente la función que aparece en la
% función objetivo f(X). Los valores de estas funciones vienen de la
% función simularPerfil