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
        cD0Despegue
        cD0Crucero
        cD0Descenso
        k
        S
        Cla

        % Parámetros de motor
        Tsl         %Empuje a nivel del mar
        TSFCsl      %Consumo específico a nivel del mar
        nummotores  %Número de motores
        % Parametros masicos
        OEW
        MTOW
        
        % Restricciones
        FWmax
        techoDeVuelo
        vMaxCrucero
        vMinCrucero
        vMaxDespegue
        vMinDespegue
        vMaxAproximacion
        vMinAproximacion

        % Parámetros de estado y derivadas
        estado % encapsula x, h, m
        dEstado
        T 

        nombreAvion

    end

    methods
        % Constructor
        function avion = Avion(nombreAvion)
            
            avion.nombreAvion = nombreAvion;
            rutaData = strcat("Data/datos_", nombreAvion, ".mat");
            try
                dataCargada = load(rutaData);
                
            catch ME
                error(strcat("No se pueden cargar los datos, revisa la ruta de " , nombreAvion , "."))
            end
            
            % asignación de variables del avión, si falta alguna o está mal
            % escrita salta un error
            try 
                parametros = dataCargada.parametros;
                fronteras = dataCargada.fronteras;
                
                % parametros aerodinamicos
                avion.cD0Despegue = parametros.cD0Despegue;
                avion.cD0Crucero = parametros.cD0Crucero;
                avion.cD0Descenso = parametros.cD0Descenso;
                avion.k = parametros.k;

                avion.S = parametros.S;

                avion.Cla = parametros.Cla;
                
                % parametros del motor
                avion.Tsl = parametros.Tsl;
                avion.TSFCsl = parametros.TSFCsl;
                avion.nummotores = parametros.nummotores;

                % parametros masicos
                avion.MTOW = parametros.MTOW;
                avion.OEW = parametros.OEW;

                % restricciones de vuelo
                avion.FWmax = fronteras.FWmax;
                avion.techoDeVuelo = fronteras.techoDeVuelo;
                avion.vMaxCrucero = fronteras.vMaxCrucero;
                avion.vMinCrucero = fronteras.vMinCrucero;
                avion.vMaxDespegue = fronteras.vMaxDespegue;
                avion.vMinDespegue = fronteras.vMinDespegue;
                avion.vMaxAproximacion = fronteras.vMaxAproximacion;
                avion.vMinAproximacion = fronteras.vMinAproximacion;

                % el estado se declarará una vez cargue la simulacion con
                % las variables de decisión

            catch ME
                error(strcat("Revisa que los campos de datos en el avión ", nombreAvion, " estén todos y que su nombre sea consistente."))
            end        
        end
        % T = calcularAeroyFuerzas(avion);
        % dm = calcularMotor(avion, estado, T);
    end
end