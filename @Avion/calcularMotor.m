
function [dmdt,Restriccion] = calcularMotor(Avion, estado, T)
%condiciones de vuelo
Alt = estado.h;
[z,d] = ISA(Alt); 
Mach = estado.V / (340*sqrt(z));

Restriccion = 0;

%%

params = struct;

% Condiciones de vuelo y empuje requerido
params.T_0 = 288.15*z;
params.P_0 = 101325*d;
params.v_0 = estado.V;
params.E_obj = T/Avion.nummotores/0.9; %Paso de empuje ins a no-ins

%Datos del avión
params.alpha_4 = Avion.alpha_4;
params.alpha_45 = Avion.alpha_45;
params.BPR = Avion.BPR;
params.A_8 = Avion.A_8;
params.A_18 = Avion.A_18;
%params.k_f = k_f;

%Constantes

    %Poder calorífico queroseno
    L = 43e6;
    %Constantes del aire
    l_c = 1.4; %gamma del aire
    params.l_c = l_c;
    l_e = 1.33; %gamma del aire caliente
    params.l_e = l_e;
    params.cp_c = 1004.5;
    params.cp_e = 1150; 
    params.R = 287;

    params.crit_c = ((l_c+1)/2)^(l_c/(l_c-1));
    params.crit_e = ((l_e+1)/2)^(l_e/(l_e-1));
    %Rendimientos
    n02 = 0.99;
    params.n_c = 0.87;
    params.n_t = 0.94;
    params.pi_34 = 0.97;
    params.n_f = 0.9;


%Datos entrada al motor
params.T_0t = params.T_0*(1+(l_c-1)/2*Mach^2);
pt0 = params.P_0*(1+(l_c-1)/2*Mach^2)^(l_c/(l_c-1));
params.P_0t = n02*pt0;


opts = optimoptions('fsolve','Display','off');

T4t_0 = 1200;
T4t_sol = fsolve( @(T4t) F(T4t,params), T4t_0, opts);

[~,T3t, G_p] = F(T4t_sol,params);

f = (params.cp_e*T4t_sol-params.cp_c*T3t)/(L-params.cp_e*T4t_sol);
c = abs(f*G_p);

if c<0 || c>4  %Por si acaso se va de madre

    TSFC= (0.45+ 0.54*Mach) * sqrt(z);

    dmdt = max(0, T*TSFC/3600/9.8);
else
    dmdt = c*Avion.nummotores;
end


end

function [r, T_3t, G_p]=F(T_4t,p)

l_c = p.l_c;
l_e = p.l_e;

%Turbinas Bloqueadas

T_45t=T_4t/p.alpha_4;
T_5t=T_45t/p.alpha_45;
%T_5t = T_45t - p.BPR*p.k_f*T_4t/p.cp_e;

% Compresión del fan

T_2t=p.cp_e/p.cp_c*(1/(1+p.BPR))*(T_45t-T_5t)+p.T_0t; %T2t=T13t en este caso
P_2t=p.P_0t*((T_2t/p.T_0t-1)*p.n_f + 1)^(l_c/(l_c-1));

T_3t=p.cp_e/p.cp_c * (T_4t-T_45t)+T_2t;
P_3t = P_2t*((T_3t/T_2t-1)*p.n_c + 1)^(l_c/(l_c-1));
P_4t=P_3t * p.pi_34;

P_45t = P_4t * (1-(1-T_45t/T_4t)/p.n_t)^(l_e/(l_e-1));
P_5t = P_45t * (1-(1-T_5t/T_45t)/p.n_t)^(l_e/(l_e-1));
% P_45t=P_4t * (1/p.alpha_4)^(l_e/(l_e-1))
% P_5t=P_45t*(1/p.alpha_45)^(l_e/(l_e-1))

% Condiciones de salida en función de si las toberas están bloqueadas

if P_2t/p.P_0 < p.crit_c
    P_18=p.P_0;
    T_18 = T_2t*(P_18/P_2t)^((l_c-1)/l_c);
    M18 = sqrt(2/(l_c-1)*(T_2t/T_18-1)); 
else
    M18 = 1;
    P_18 = P_2t/(1+(l_c-1)*0.5*M18^2)^(l_c/(l_c-1));
    T_18 = T_2t*(P_18/P_2t)^((l_c-1)/l_c);
end

if P_5t/p.P_0 < p.crit_e
    P_8=p.P_0;
    T_8 = T_5t*(P_8/P_5t)^((l_e-1)/l_e);
    M8 = sqrt(2/(l_e-1)*(T_5t/T_8-1));
else
    M8=1;
    P_8 = P_5t/(1+(l_e-1)/2*M8^2)^(l_e/(l_e-1));
    T_8 = T_5t*(P_8/P_5t)^((l_e-1)/l_e);
end

%Calculamos densidad y velocidad de salida
rho_18=P_18/(p.R*T_18);
v_18=M18*sqrt(l_c*p.R*T_18);
rho_8=P_8/(p.R*T_8);
v_8=M8*sqrt(l_e*p.R*T_8);

%Calculamos gastos
G_II=p.A_18*v_18*rho_18;
G_I=p.A_8*v_8*rho_8;
%Calculamos empujes
E_II=G_II*(v_18-p.v_0)+p.A_18*(P_18-p.P_0);
E_I=G_I*(v_8-p.v_0)+p.A_8*(P_8-p.P_0);
E_calc=E_I+E_II;

%PG4 = G_I*sqrt(T_4t)/P_4t
G_p = abs(G_I);
r = E_calc - p.E_obj;
end


function [z,d] = ISA(h)

        if h <= 11000
     		z = 1-0.0065/288.15*h;
     		d = z^5.255876;
 		else
     		z = 216.65/288.15;
     		d = (22632.1/101325)*exp(-0.0001577*(h-11000));
        end
end
