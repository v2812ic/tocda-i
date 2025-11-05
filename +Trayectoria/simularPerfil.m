% Inputs:
%   (variables desempaquetadas de evaluarVuelo)
%   dist_fases   (vector 1x4) Distancias horizontales de las fases 1-4 (m)
%   Notar que la dist_5 se puede sacar como distancia_total -
%   sum(dist_fases)
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
% Esta función es el calculador de la masa final del avión, y del tiempo. A
% partir de los datos que le llegan de evaluarVuelo, le va a decir a
% evaluarVuelo los resultados físicos del vuelo en sí. La diferencia entre
% evaluarVuelo y simularPerfil es que evaluarVuelo es el contenedor del
% problema de optimización, y simularPerfil es el que le da los resultados.
% evaluarVuelo necesita datos del vuelo? Pues llama a simularPerfil y
% simularPerfil se lo devuelve. 
% Para la simulación se tiene que tener en cuenta que la masa va cambiando
% en cada fase, por lo que este script incluye el uso de ode45.
% ode45 es una función que, dadas unas condiciones iniciales, calcula las
% condiciones finales siguiendo una edo (por el metodo de rungekutta, que
% va bastante chetado). A ode45 hay que llamarle una vez por fase, y
% detectar el fin de la fase con el archivo de detenerEnDistancia para
% saltar a la siguiente
% Necesita un vector de estado (x, h, m) - distancia,
% altura, masa y sus derivadas. Para obtener sus derivadas se recurre a las
% funciones de equilibrio de fuerzas y dinamicaDeVuelo en cada instante. 