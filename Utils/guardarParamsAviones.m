%% PLANTILLA PARA GUARDAR LOS DATOS DE CADA AVION
% TODOS LOS PARAMETROS EN SISTEMA INTERNACIONAL 

% NOMBRE DEL AVIÓN 
nombreAvion = "A320";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 1.2;
parametros.cD0Crucero = 1.2;
parametros.cD0Descenso = 1.2;
parametros.k = 1.2;

parametros.S = 122.6; %superficie alar m2

% Parámetros de motor
parametros.Tsl = 120e3;
parametros.TSFCsl = 1e-5;
parametros.nummotores = 2;

% Parametros masicos (kg)
parametros.OEW = 1e5;
parametros.MTOW = 1e6;

% Restricciones
fronteras.FWmax = 1e4; % (kg)
fronteras.techoDeVuelo = 1e5; % (m)
fronteras.vMaxCrucero = 50; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 50;
fronteras.vMaxDespegue = 50;
fronteras.vMinDespegue = 50;
fronteras.vMaxAproximacion = 50;
fronteras.vMinAproximacion = 50;

ruta = strcat("Data/datos_", nombreAvion, ".mat");

save(ruta, 'parametros', 'fronteras');