%% PLANTILLA PARA GUARDAR LOS DATOS COMUNES A TODOS LOS AVIONES

parametros.h_origen = 0; % (m)
parametros.h_destino = 0;
parametros.distancia = 1e5;

parametros.PL = 1e2;
parametros.seguridadFuel = 1.1; %> 1!!

fronteras.maxTasaAscenso = 0.5; % (rad)
fronteras.maxTasaDescenso = -0.5; % Negativo !!

fronteras.x1Max = 1e5; % (m)
fronteras.x2Max = 1e5;
fronteras.x3Max = 1e5;
fronteras.x4Max = 1e5;

fronteras.x1Min = 1e2;
fronteras.x2Min = 1e2;
fronteras.x3Min = 1e2;
fronteras.x4Min = 1e2;

fronteras.x5Max = 1e5;
fronteras.x5Min = 1e2;

fronteras.hCruceroMin = 1e3;

ruta = fullfile('Data/restriccionesGenerales.mat');


save(ruta, 'parametros', 'fronteras');