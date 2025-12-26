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
parametros.Tsl = 120e3;
parametros.TSFCsl = 1e-5;
parametros.nummotores = 1;
parametros.Tmax = 60000;


% Parametros masicos (kg)
parametros.OEW = 10600;
parametros.MTOW = 17622;

% Restricciones
fronteras.FWmax = 9000; % (kg)
fronteras.techoDeVuelo = 40000; % (m)
fronteras.vMaxCrucero = 245; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 60;
fronteras.vMaxDespegue = 245;
fronteras.vMinDespegue = 60;
fronteras.vMaxAproximacion = 245;
fronteras.vMinAproximacion = 60;
ruta = strcat("Data/datos_", nombreAvion, ".mat");

save(ruta, 'parametros', 'fronteras');