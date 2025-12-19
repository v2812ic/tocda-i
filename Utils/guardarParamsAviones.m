%% PLANTILLA PARA GUARDAR LOS DATOS DE CADA AVION
% TODOS LOS PARAMETROS EN SISTEMA INTERNACIONAL 

% NOMBRE DEL AVIÓN 

nombreAvion = "Concorde";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.075;
parametros.cD0Crucero = 0.020;
parametros.cD0Descenso = 0.026;
parametros.k = 0.22;


% Parámetros de motor

% Parametros masicos (kg)
parametros.OEW = 78700;
parametros.MTOW = 185000;

% Restricciones
fronteras.FWmax = 95700; % (kg)
fronteras.techoDeVuelo = 18300; % (m)
fronteras.vMaxCrucero = 610 ; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 592;
fronteras.vMaxDespegue = 87;
fronteras.vMinDespegue = 77;
fronteras.vMaxAproximacion = 113;
fronteras.vMinAproximacion = 103;
ruta = strcat("Data/datos_", nombreAvion, ".mat");

save(ruta, 'parametros', 'fronteras');