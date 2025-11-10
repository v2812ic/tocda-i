% Inputs:
%   nombreAvion (string) Nombre del avión (ej. 'A320'). Carga el archivo de
%                        datos correspondiente (ej. /Data/datos_A320.mat).
%
% Outputs:
%   obj         (objeto) La instancia de la clase 'Avion' con todas las
%   funciones y datos cargados

% Ademas avion va a guardar las variables instantaneas que vamos a coger
% para el estado de la EDO

classdef Avion < handle
    properties

        % Parámetros aerodinámicos
        cD0_despegue
        cD0_crucero
        cD0_descenso
        k

        % Parámetros de motor

        % Parámetros de estado y derivadas
        estado % encapsula x, h, m
        dEstado
        T 
        % L y D solo son necesarios para equilibrar las ecuaciones de
        % fuerzas, T es necesario guardarlo para pasarselo a la dinámica
        % del vuelo y que calculamotor lo use para sacar el consumo

        nombreAvion

    end

    methods
        % Constructor
        function avion = Avion(nombreAvion)
            
            avion.nombreAvion = nombreAvion;
            rutaData = fullfile(['Data/datos_' nombreAvion '.mat']);

            try
                dataCargada = load(rutaData);
            catch ME
                error("No se pueden cargar los datos, revisa la ruta de " + nombreavion + ".")
            end
            
            % asignación de variables del avión, si falta alguna o está mal
            % escrita salta un error
            try 
                avion.cD0_despegue = dataCargada.cD0_despegue;
                avion.cD0_crucero = dataCargada.cD0_crucero;
                avion.cD0_descenso = dataCargada.cD0_descenso;
                avion.k = dataCargada.k;

            catch ME
                error("Revisa que los campos de datos en el avión " + nombreavion + " estén todos y que su nombre sea consistente.")
            end        
        end
        T = calcularAeroyFuerzas(avion);
        dm = calcularMotor(avion, estado);
    end
end