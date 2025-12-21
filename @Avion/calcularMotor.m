% Inputs:
%   obj         (objeto) La instancia de la clase 'Avion' (inyectado)
%   V           (scalar) Velocidad actual (m/s)
%   h           (scalar) Altitud actual (m)
%   T           (scalar) empuje deseado
%
% Outputs:
%   m_dot       (scalar) Flujo de combustible (kg/s)


function dmdt = calcularMotor(Avion, estado, T)
%condiciones de vuelo
Alt = estado.h;
z = ISA(Alt); 
Mach = estado.V / (340*sqrt(z)); %Cálculo del Mach de vuelo
Thrust_Req = T/Avion.nummotores;  %Empuje requerido

% Cambio de los parámetros del motor 
% Fmax=> Maximum sea-level static thrust (N)
% SFC=> Sea-level static TSFC (kg/(N.s))
% tau=> Fastest engine time constant (s) controla lo rápido que se adapta a
% cambios de throttle. PONERLO BAJO PARA QUE CONVERJA
% Nt=> Ratio of installed to unstalled thrust DEJARLO COMO ESTÁ
% ic_source=> initial thrust source ('Internal' o 'External')
% IC=> initial thrust (N) (solo aplica si ic_source = 'Eternal')

params = struct('Fmax',Avion.Tsl,'SFC',Avion.SFCsl);

apply_engine_params(params);

% Configuración de la simulación

opts.Model     = 'Turbofan_Model';  %Nombre del modelo de Simulink
opts.StopTime  = '1';   %Tiempo de simulación
opts.ThrustSig = 'Thrust_N';    %Nombre de la señal de empuje
opts.FuelSig   = 'FuelFlow_kgps';   %Nombre de la señal de consumo

% Llamada a la función
[Thrust_data,Fuel_data] = simulate_once(Mach, Alt, opts);

Fuel_data = squeeze(Fuel_data);
[Thrust_data , idx] = unique(Thrust_data);
Fuel_data = Fuel_data(idx);



dmdt = interp1(Thrust_data, Fuel_data, Thrust_Req)*Avion.nummotores;


%Aviso si se le está pidiendo más empuje del que da el motor
if Thrust_Req>max(Thrust_data)

    warning('El motor no es capaz de dar el suficiente empuje. Empuje requerido: %.1f N, Empuje máximo %.1f N', ...
        Thrust_Req, max(Thrust_data))
    warning('El valor del consumo se ha conseguido por extrapolación')

    dmdt = interp1(Thrust_data, Fuel_data, Thrust_Req, 'linear', 'extrap');
end

end



% ======= funciones auxiliares =======

function [ThrustN, Fuel] = simulate_once(MachVal, AltVal, opts)

    % Prepara la entrada de simulación
    in = Simulink.SimulationInput(opts.Model);

    % --- From Workspace requiere timeseries ---
    %  Usamos el mismo StopTime de la simulación para que sea constante en el tiempo
    tEnd = str2double(opts.StopTime);
    if isnan(tEnd), tEnd = 0.0001; end
    t = [0 tEnd];

    Throttle_ts = timeseries([0 1], t);
    Mach_ts     = timeseries([MachVal MachVal], t);
    Alt_ts      = timeseries([AltVal AltVal], t);

    % Variables disponibles para el modelo
    in = in.setVariable('Throttle', Throttle_ts);
    in = in.setVariable('Mach',     Mach_ts);
    in = in.setVariable('Alt',      Alt_ts);

    % --- Parámetros del modelo: aquí se fija el modo correcto ---
    in = in.setModelParameter('StopTime', opts.StopTime);

    % Ejecuta la simulación
    out = sim(in);

    % --- Lectura robusta de salidas ---
    ThrustN = NaN; Fuel = NaN;

    %Obtenemos los datos de la simulación
    if isprop(out, opts.ThrustSig)
        tsT = out.(opts.ThrustSig);
        if isa(tsT, 'timeseries') && ~isempty(tsT.Data)
            ThrustN = tsT.Data;
        end
    end
    if isprop(out, opts.FuelSig)
        tsF = out.(opts.FuelSig);
        if isa(tsF, 'timeseries') && ~isempty(tsF.Data)
            Fuel = tsF.Data;
        end
    end

end

function apply_engine_params(p)
    model = 'Turbofan_Model';

   
    % 1) Cargar el modelo si no está cargado
    if ~bdIsLoaded(model)
        load_system(model);   % no abre ventana
    end

    % 2) Intentar encontrar el bloque por distintas claves
    blk = '';  % ruta completa al bloque dentro del modelo

    % (a) Por referencia a la librería donde vive el bloque
    hits = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', ...
        'ReferenceBlock','aerolibpropulsion2/Turbofan Engine System');
    if ~isempty(hits); blk = hits{1}; end

    % (b) Si no, por MaskType típico
    if isempty(blk)
        hits = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', ...
            'MaskType','Turbofan Engine System');
        if ~isempty(hits); blk = hits{1}; end
    end

    % (c) Si no, por nombre aproximado del bloque dentro del modelo
    if isempty(blk)
        hits = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', ...
            'Name','Turbofan Engine System');
        if ~isempty(hits); blk = hits{1}; end
    end

    % (d) Último intento: ¿lo llamaste "Turbofan Engine"?
    if isempty(blk)
        hits = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', ...
            'Name','Turbofan Engine');
        if ~isempty(hits); blk = hits{1}; end
    end

    % Si sigue vacío, avisar con info útil
    if isempty(blk)
        error(['No encuentro el bloque Turbofan dentro de ', model, ...
               '. Abre el modelo y dime el nombre exacto del bloque o selecciona el bloque y usa: ', ...
               'getfullname(gcb)']);
    end


    % 3) Aplicar parámetros
    f = @(name,val) set_param(blk, name, num2str(val));

    if isfield(p,'Fmax'); f('Fmax', p.Fmax); end     % N
    if isfield(p,'SFC');  f('SFC',  p.SFC);  end     % kg/(N*h)
   
    save_system(model);

end


function [z,d] = ISA(h)

        if h <= 11000
     		z = 1-0.0065/288.15*h;
     		d = z^5.255876;
 		else
     		z = 216.65/288.15;
     		d = (22632.1/101325)*exp(-0.0001577*(h-11000));
        end

end

