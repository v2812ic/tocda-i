


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