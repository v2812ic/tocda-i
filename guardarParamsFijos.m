%% PLANTILLA PARA GUARDAR LOS DATOS COMUNES A TODOS LOS AVIONES

parametros.h_origen = 0; % (m)
parametros.h_destino = 0;
parametros.distancia = 1e6;

fronteras.maxTasaAscenso = 100; % (m/s)
fronteras.minTasaAscenso = 10;
fronteras.maxTasaDescenso = 100; 
fronteras.minTasaDescenso = 10; 

fronteras.x1Max = 1e3; % (m)
fronteras.x2Max = 1e5;
fronteras.x3Max = 1e6;
fronteras.x4Max = 1e6;

fronteras.x1Min = 1e2;
fronteras.x2Min = 1e2;
fronteras.x3Min = 1e2;
fronteras.x4Min = 1e2;

ruta = fullfile('Data/restriccionesGenerales.mat');

save(ruta, 'parametros', 'fronteras');