%% PLANTILLA PARA GUARDAR LOS DATOS DE CADA AVION
% TODOS LOS PARAMETROS EN SISTEMA INTERNACIONAL 

% NOMBRE DEL AVIÓN 
nombreAvion = "BC300";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.011;
parametros.cD0Crucero = 0.011;
parametros.cD0Descenso = 0.015;
parametros.k = 0.043;

parametros.S = 122.6; 
parametros.Cla = 6;

% Parámetros de motor
E=120e3;
T4t_max = 1750;
pi_c = 30;
parametros.BPR = 11;
pi_f = 1.4;
[alpha_4, alpha_45, A_8, A_18] = Calculo_parametros_motor(E,T4t_max,pi_c,parametros.BPR,pi_f);

parametros.alpha_4 = alpha_4;
parametros.alpha_45 = alpha_45;
parametros.A_8 = A_8;
parametros.A_18 = A_18;

parametros.Tsl = 120e3;
parametros.TSFCsl = 1e-5;
parametros.nummotores = 2;
parametros.Tmax = 60000;


% Parametros masicos (kg)
parametros.OEW = 10600;
parametros.MTOW = 17622;

% Restricciones
fronteras.FWmax = 9000; % (kg)
fronteras.techoDeVuelo = 13716; % (m)
fronteras.vMaxCrucero = 245; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 60;
fronteras.vMaxDespegue = 245;
fronteras.vMinDespegue = 60;
fronteras.vMaxAproximacion = 245;
fronteras.vMinAproximacion = 60;
ruta = strcat("Data/datos_", nombreAvion, ".mat");

save(ruta, 'parametros', 'fronteras');