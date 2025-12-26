%% PLANTILLA PARA GUARDAR LOS DATOS COMUNES A TODOS LOS AVIONES

parametros.h_origen = 0; % (m)
parametros.h_destino = 0;
parametros.distancia = 4e6;

parametros.PL = 1e2;
parametros.seguridadFuel = 1.1; %> 1!!

parametros.xRef = [1e5, 1e6, 1e2, 1e2, 1e2, 1e4, 1e4, 1e4];


fronteras.maxTasaAscenso = deg2rad(8);     % Angulos máximos de ascenso y descenso(deg)
fronteras.minTasaAscenso = deg2rad(3);
fronteras.minTasaDescenso = -deg2rad(3);
fronteras.maxTasaDescenso = -deg2rad(8);  

fronteras.hCruceroMin = 8e3;

fronteras.x1Max = parametros.distancia; % (m)
fronteras.x2Max = parametros.distancia;

fronteras.x1Min = fronteras.hCruceroMin/tan(fronteras.maxTasaAscenso);
fronteras.x2Min = 0;

fronteras.x3Max = parametros.distancia;
fronteras.x3Min = -fronteras.hCruceroMin/tan(fronteras.maxTasaDescenso);

ruta = fullfile('Data/restriccionesGenerales.mat');


save(ruta, 'parametros', 'fronteras');