%Código para calcular datos del motor a partir de:
%Empuje
%T4t
%Relación de compresión del compresor 2-3
%Relación de compresión del fan
%Relación de derivación BPR
%Rendimientos

function [alpha_4,alpha_45,A_8,A_18] = Calculo_parametros_motor(E,T4t,pi_c,BPR,pi_f)
%Datos que definen el turbofan
% pi_f = 1.4;
% BPR = 11;
% T4t = 1750;
% pi_c = 30;
% 
% E = 120e3;

%Rendimientos
n02 = 0.99;
n_c = 0.9;
n_t = 0.95;
n_34 = 0.97;
n_f = 0.9;

%Condiciones a nivel del mar
Mach =0;
T0 = 288.15;
p0 = 101325;

L = 43e6;

l_c = 1.4; %gamma del aire
l_e = 1.33; %gamma del aire caliente
cp_c = 1004.5;
cp_e = 1150; 

%Datos para comprobar toberas criticas
crit_hot  = ((l_e+1)/2)^(l_e/(l_e-1));
crit_cold = ((l_c+1)/2)^(l_c/(l_c-1));

V0 = Mach*sqrt(l_c*287*T0);

%Datos entrada al motor

pt0 = p0*(1+(l_c-1)/2*Mach^2)^(l_c/(l_c-1));
p2t = n02*pt0;
T2t = T0*(1+(l_c-1)/2*Mach^2);

%Flujo secundario
tao_f = cp_c*T2t*(pi_f^((l_c-1)/l_c) - 1)/n_f; %= cp_e*(T45t-T5t)

T13t = tao_f/cp_c + T2t;
p13t = p2t*pi_f;

%Está bloqueado la tobera del secundario?
NPR_sec = p13t/p0;

%Ligadura Compresor-Turbina

%T45t = T4t - cp_c/cp_e*T2t*(pi_c^((l_c-1)/l_c)-1)/(n_c*n_t);

T3t = T13t*(1 + (pi_c^((l_c-1)/l_c)-1)/n_c);
T45t = T4t - (cp_c/cp_e)*(T3t - T13t);

%Ligadura Turbina del fan - Fan

T5t = T45t - (1+BPR)*tao_f/cp_e;

p4t = n_34*pi_c*p13t;
p45t = p4t * (1-(1-T45t/T4t)/n_t)^(l_e/(l_e-1));
p5t =p45t*(1-(T45t-T5t)/T45t/n_t)^(l_e/(l_e-1));

%Está bloqueada la tobera del primario?
NPR_pri = p5t/p0;

%Empuje 

if NPR_pri<crit_hot
    p8=p0;
    T8 = T5t/(p5t/p8)^((l_e-1)/l_e);
    M8 = sqrt(2/(l_e-1)*(T5t/T8-1));
    %e_pri = sqrt(2*cp_e*(T5t-T8))-V0; %Empuje específico

else
    %A8 = sqrt(287*T5t)*G_p/Gamma_e/p5t;
    M8=1;
    p8 =p5t/(1+(l_e-1)*0.5*M8)^(l_e/(l_e-1));
    T8 = T5t/(p5t/p8)^((l_e-1)/l_e);
    
    %e_pri = sqrt(2*cp_e*(T5t-T8))-V0 + A_8*(p8 - p0);

end

if NPR_sec<crit_cold
    p18=p0;
    T18 = T13t*(p18/p13t)^((l_c-1)/l_c);
    M18 = sqrt(2/(l_c-1)*(T13t/T18-1)); 
    %e_sec = BPR*(sqrt(2*cp_c*(T13t-T18))-V0);

else

    M18 = 1;
    p18 = p13t/(1+(l_c-1)*0.5*M18^2)^(l_c/(l_c-1));
    T18 = T13t*(p18/p13t)^((l_c-1)/l_c);
   
    %A18 = sqrt(287*T13t)*BPR*G_p/Gamma_c/p13t;
    
    %e_sec = BPR*(sqrt(2*cp_c*(T13t-T18))-V0) + A_18*(p18-p0); 
end

L_pri = gamma_M_l(M8,l_e);
L_sec = gamma_M_l(M18,l_c);
%A8 = G_p*sqrt(287*T5t)/L_pri/p5t;
%A18 = BPR*G_p*sqrt(287*T13t)/L_sec/p13t;

fun = @(G_p)(G_p*(sqrt(2*cp_e*(T5t-T8))-V0 +BPR*(sqrt(2*cp_c*(T13t-T18))-V0)) ...
    + G_p*sqrt(287*T5t)/L_pri/p5t*(p8 - p0) + ...
     + BPR*G_p*sqrt(287*T13t)/L_sec/p13t*(p18-p0)-E);

G_p_0 = 20;

G_p_sol = fsolve(fun,G_p_0)
%%Obtención de parámetros constantes en actuaciones
   
%Parámetro de gasto en las turbinas

PG_4 = G_p_sol*sqrt(T4t)/p4t;

PG_45 = G_p_sol*sqrt(T45t)/p45t;

k_c = cp_c*T2t/(n_c*T4t)*(pi_c^((l_c-1)/l_c)-1);
k_f = tao_f/T4t;

%Áreas de interés

A_8 = sqrt(287*T5t)*G_p_sol/L_pri/p5t
A_18 = sqrt(287*T13t)*BPR*G_p_sol/L_sec/p13t

% Cálculo de consumos 
%(para ver que sea coherente)
f = (cp_e*T4t-cp_c*T3t)/(L-cp_e*T4t);
c = f*G_p_sol;

TSFC = c/E;

alpha_4 = T4t/T45t
%alpha_p = p4t/p45t;
alpha_45 = T45t/T5t

function [Gamma] = gamma_M_l(M,l)
Gamma = M*sqrt(l)*(1+(l-1)/2*M^2)^(-(l+1)/(2*(l-1)));
end

end