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

function Resultado = simularPerfil(Dist, vel, altitud, comb, Param, fronteras, Avion)

i = 0;
j = 1;
g=1;
t0 = 0;
Resultado = struct;
TOW = Avion.OEW+Param.PL+comb;
Resultado.violacionRestricciones = 0;

Dist(5) = Param.distancia-sum(Dist);
gamma_f = [atan((altitud(1)-Param.h_origen)/Dist(1)) 0 atan((altitud(2)-altitud(1))/Dist(3)) 0 -atan((altitud(2)-Param.h_destino)/Dist(5))];
V_asc = vel.*sin(gamma_f);

if (V_asc(1)>fronteras.maxTasaAscenso) || (V_asc(1) < fronteras.minTasaAscenso)
    Resultado.violacionRestricciones = 1;
    g = 8;

elseif (V_asc(3)>fronteras.maxTasaAscenso) || (V_asc(3) < fronteras.minTasaAscenso)
    Resultado.violacionRestricciones = 1;
    g = 8;

elseif (abs(V_asc(5))>fronteras.maxTasaDescenso) || (abs(V_asc(5))<fronteras.minTasaDescenso)
    Resultado.violacionRestricciones = 1;
    g = 8;
elseif TOW>Avion.MTOW
    Resultado.violacionRestricciones = 1;
    g = 8;

end

%Tiempo de cada fase
t_fase = Dist./(vel.*cos(gamma_f));

%Estructura de salida
Ins = struct;



%1 fase
i = 1+i;

%Inicia las variables para la primera fase
Ins.comb_rest(i,j) = comb;
Ins.comb_cons(i,j) = 0;
Ins.dist(i,j) = 0;
Ins.alt(i,j) = Param.h_origen;
Ins.t(i,j) = t0;
Ins.peso(i,j) = TOW;

%Vector de estado
S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];

j = 1+j;
while g==i

    
    v = vel(i);
    gamma = gamma_f(i);

    %Calcula las derivadas de estado
    dS = dinamicaDeVuelo(Ins.t(i,j-1), S, v, gamma);

    odefun = dS(3);
    tspan = [0 t_fase(i)/100];
    %ini = Avion.properties.estado;
    ini = 0;
    %Resuelve la ecuación diferencial para el gasto
    [t, y] = ode45(@(t,y)odefun, tspan, ini);

    
    %Calcula las salidas
    Ins.t(i,j) = Ins.t(i,j-1) + t(end);
    Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
    Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
    Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
    Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
    Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
    
    if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
        Resultado.violacionRestricciones = 1;
        g=8;
        break
    end


    %Vector de estado
    S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];

    
    %Analiza si termina la fase
    g = detenerEnDistancia(Dist, Ins.dist(i,j));

    j = 1+j;

end

Ins.dist_total = Ins.dist;
Ins.t_total = Ins.t;


%2 fase
j = 1;
i = 1+i;


Ins.comb_rest(i,j) = Ins.comb_rest(i-1,end);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,end);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, end);
Ins.alt(i,j) = Ins.alt(i-1,end);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,end);
Ins.peso(i,j) = Ins.peso(i-1,end);




j = 1+j;



while g==i

    v = vel(i);
    gamma = gamma_f(i);

    dS = dinamicaDeVuelo(Ins.t(i,j-1), S, v, gamma);

    odefun = dS(3);
    tspan = [0 t_fase(i)/100];
    ini = 0;

    [t, y] = ode45(@(t,y)odefun, tspan, ini);



    Ins.t(i,j) = Ins.t(i,j-1) + t(end);
    Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
    Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
    Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
    Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
    Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
    Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
    Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
    
    if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
        Resultado.violacionRestricciones = 1;
        g=8;
        break
    end



    S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];



    g = detenerEnDistancia(Dist, Ins.dist_total(i,j));

    j = 1+j;

end

%3 fase
j = 1;
i = 1+i;


Ins.comb_rest(i,j) = Ins.comb_rest(i-1,end);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,end);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, end);
Ins.alt(i,j) = Ins.alt(i-1,end);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,end);
Ins.peso(i,j) = Ins.peso(i-1,end);




j = 1+j;



while g==i

    v = vel(i);
    gamma = gamma_f(i);

    dS = dinamicaDeVuelo(Ins.t(i,j-1), S, v, gamma);

    odefun = dS(3);
    tspan = [0 t_fase(i)/100];
    ini = 0;

    [t, y] = ode45(@(t,y)odefun, tspan, ini);



    Ins.t(i,j) = Ins.t(i,j-1) + t(end);
    Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
    Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
    Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
    Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
    Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
    Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
    Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
  

    if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
        Resultado.violacionRestricciones = 1;
        g=8;
        break
    end



    S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];



    g = detenerEnDistancia(Dist, Ins.dist_total(i,j));

    j = 1+j;

end

%4 fase
j = 1;
i = 1+i;


Ins.comb_rest(i,j) = Ins.comb_rest(i-1,end);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,end);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, end);
Ins.alt(i,j) = Ins.alt(i-1,end);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,end);
Ins.peso(i,j) = Ins.peso(i-1,end);




j = 1+j;



while g==i

    v = vel(i);
    gamma = gamma_f(i);

    dS = dinamicaDeVuelo(Ins.t(i,j-1), S, v, gamma);

    odefun = dS(3);
    tspan = [0 t_fase(i)/100];
    ini = 0;

    [t, y] = ode45(@(t,y)odefun, tspan, ini);



    Ins.t(i,j) = Ins.t(i,j-1) + t(end);
    Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
    Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
    Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
    Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
    Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
    Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
    Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
    

    if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
        Resultado.violacionRestricciones = 1;
        g=8;
        break
    end



    S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];



    g = detenerEnDistancia(Dist, Ins.dist_total(i,j));

    j = 1+j;

end


%5
j = 1;
i = 1+i;


Ins.comb_rest(i,j) = Ins.comb_rest(i-1,end);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,end);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, end);
Ins.alt(i,j) = Ins.alt(i-1,end);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,end);
Ins.peso(i,j) = Ins.peso(i-1,end);




j = 1+j;



while g==i

    v = vel(i);
    gamma = gamma_f(i);

    dS = dinamicaDeVuelo(Ins.t(i,j-1), S, v, gamma);

    odefun = dS(3);
    tspan = [0 t_fase(i)/100];
    ini = 0;

    [t, y] = ode45(@(t,y)odefun, tspan, ini);



    Ins.t(i,j) = Ins.t(i,j-1) + t(end);
    Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
    Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
    Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
    Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
    Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
    Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
    Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
    

    if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
        Resultado.violacionRestricciones = 1;
        g=8;
        break
    end



    S = [Ins.dist(i,j), Ins.alt(i,j), Ins.peso(i,j)];



    g = detenerEnDistancia(Dist, Ins.dist_total(i,j));

    j = 1+j;

end

Resultado.tiempoTotal = Ins.t_total(end, end);
Resultado.combustibleConsumido = Ins.comb_cons(end,end);

end
