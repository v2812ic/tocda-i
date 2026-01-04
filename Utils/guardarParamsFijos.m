%% PLANTILLA PARA GUARDAR LOS DATOS COMUNES A TODOS LOS AVIONES

parametros.h_origen = 0; % (m)
parametros.h_destino = 0;
parametros.distancia = 4e6;

parametros.PL = 800;
parametros.seguridadFuel = 1.1; %> 1!!

parametros.xRef = [1e5, 1e6, 1e2, 1e2, 1e2, 1e4, 1e4, 1e5]; % Escalado
%intuitivo para llegar a un optimo razonable
% parametros.xRef = [14142, 1578, 31.62, 5.77, 8.16, 60.37, 471.4, 560.6]; % Escalado hessiana
%parametros.xRef = [1, 1, 1, 10, 10, 1, 1, 100];
%parametros.xRef = [1, 1, 1, 1, 1, 1, 1, 1];     %Sin escalados
parametros.xRef = Xref_nuevo';
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

fronteras.angulo_min = atan(fronteras.hCruceroMin/parametros.distancia*2);

ruta = fullfile('Data/restriccionesGenerales.mat');


save(ruta, 'parametros', 'fronteras');

% diag hessiano: 
% 
% 1.0e+05 *

    % 0.0005
    % 4.0174
    % 0.0001
    % 0.0030
    % 0.0015
    % 0.2744
    % 0.0045
    % 0.3182