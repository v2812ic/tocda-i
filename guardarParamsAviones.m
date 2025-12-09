%% PLANTILLA PARA GUARDAR LOS DATOS DE CADA AVION
% TODOS LOS PARAMETROS EN SISTEMA INTERNACIONAL 

% NOMBRE DEL AVIÓN 
nombreAvion = "B747";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 1.2;
parametros.cD0Crucero = 1.2;
parametros.cD0Descenso = 1.2;
parametros.k = 1.2;

% Parámetros de motor

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

% NOMBRE DEL AVIÓN 
nombreAvion = "Bombardier Challenger 300";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.042;
parametros.cD0Crucero = 0.011;
parametros.cD0Descenso = 0.015;
parametros.k = 0.043;

% Parámetros de motor

% Parametros masicos (kg)
parametros.OEW = 10600;
parametros.MTOW = 17622;

% Restricciones
fronteras.FWmax = 6400; % (kg)
fronteras.techoDeVuelo = 13.716; % (m)
fronteras.vMaxCrucero = 245; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 236;
fronteras.vMaxDespegue = 62;
fronteras.vMinDespegue = 72;
fronteras.vMaxAproximacion = 57;
fronteras.vMinAproximacion = 67;

% NOMBRE DEL AVIÓN 
nombreAvion = "A380-800";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.055;
parametros.cD0Crucero = 0.014;
parametros.cD0Descenso = 0.017;
parametros.k = 0.043;

% Parámetros de motor

% Parametros masicos (kg)
parametros.OEW = 277000;
parametros.MTOW = 575000;

% Restricciones
fronteras.FWmax = 250000; % (kg)
fronteras.techoDeVuelo = 13000; % (m)
fronteras.vMaxCrucero = 251; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 236;
fronteras.vMaxDespegue = 77;
fronteras.vMinDespegue = 93;
fronteras.vMaxAproximacion = 72;
fronteras.vMinAproximacion = 77;

% NOMBRE DEL AVIÓN 
nombreAvion = "A350-1000";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.040;
parametros.cD0Crucero = 0.009;
parametros.cD0Descenso = 0.017;
parametros.k = 0.037;

% Parámetros de motor

% Parametros masicos (kg)
parametros.OEW = 206 000;
parametros.MTOW = 322 000;

% Restricciones
fronteras.FWmax = 124 000; % (kg)
fronteras.techoDeVuelo = 13000; % (m)
fronteras.vMaxCrucero = 250; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 236;
fronteras.vMaxDespegue = 77;
fronteras.vMinDespegue = 87;
fronteras.vMaxAproximacion = 72;
fronteras.vMinAproximacion = 77;

% NOMBRE DEL AVIÓN 
nombreAvion = "Concorde";

% Parámetros aerodinámicos (adim)
parametros.cD0Despegue = 0.075;
parametros.cD0Crucero = 0.020;
parametros.cD0Descenso = 0.026;
parametros.k = 0.22;

% Parámetros de motor

% Parametros masicos (kg)
parametros.OEW = 78 700;
parametros.MTOW = 185 000;

% Restricciones
fronteras.FWmax = 95 700; % (kg)
fronteras.techoDeVuelo = 18 300; % (m)
fronteras.vMaxCrucero = 592; % todas las velocidades (m/s)   
fronteras.vMinCrucero = 610;
fronteras.vMaxDespegue = 77;
fronteras.vMinDespegue = 87;
fronteras.vMaxAproximacion = 103;
fronteras.vMinAproximacion = 113;

ruta = strcat("Data/datos_", nombreAvion, ".mat");

save(ruta, 'parametros', 'fronteras');