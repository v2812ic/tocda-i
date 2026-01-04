
%% 1. CONFIGURACIÓN INICIAL Y CARGA DE DATOS
import Core.evaluarVuelo
addpath('Utils');

% CONTROL DE LA SIMULACIÓN - UNICA SECCIÓN A TOCAR

aviones = ["BC300"]; 
heuristico = false; 
gradiente = true; 
w1 = 0; % tiempo
w2 = 1; % combustible 
precisionRelEDO = 1e-5;

restriccionesGenerales = load("Data/restriccionesGenerales.mat");
parametrosFijos = restriccionesGenerales.parametros;
fronterasFijas = restriccionesGenerales.fronteras;
control.nvars = size(parametrosFijos.xRef, 2);
parametrosFijos.precisionRelEDO = precisionRelEDO;
control.heuristico = heuristico;
control.gradiente = gradiente;
control.w1 = w1;
control.w2 = w2;

%% 2. BUCLE PRINCIPAL DE OPTIMIZACIÓN
for i = 1:length(aviones)
    avionActual = aviones(i);
    fprintf("------------------------------------------------\n");
    fprintf(strcat("Comienza la optimización del avión ", avionActual, "\n\n"));
    
    % Inicialización del modelo de avión
    try
        avion = Avion(avionActual);
    catch ME
        fprintf(strcat("Deteniendo simulación para avión ", avionActual, ".\n"));
        fprintf(strcat(ME.message, "\n\n"));
        continue; 
    end
    
    % Validación de carga útil (Payload)
    if avion.MTOW - avion.OEW - parametrosFijos.PL <= 0
        fprintf('Se aborta la optimización con avión %s porque no puede cargar tanto payload.\n\n', avionActual);
        continue
    end
    
    %% 3. DEFINICIÓN DE LÍMITES Y RESTRICCIONES LINEALES
    % Cálculo de distancias máximas basadas en aerodinámica
    fronterasFijas.x1Max = avion.techoDeVuelo/tan(fronterasFijas.minTasaAscenso);
    fronterasFijas.x3Max = -avion.techoDeVuelo/tan(fronterasFijas.minTasaDescenso);
    
    % Definición de Lower y Upper Bounds (lb, ub)
    lb = [min(fronterasFijas.x1Min, parametrosFijos.distancia), min(fronterasFijas.x2Min, parametrosFijos.distancia), avion.vMinDespegue, avion.vMinCrucero, ...
        avion.vMinAproximacion, fronterasFijas.hCruceroMin, fronterasFijas.hCruceroMin,...
        0];
    ub = [min(fronterasFijas.x1Max, parametrosFijos.distancia), min(fronterasFijas.x2Max, parametrosFijos.distancia), avion.vMaxDespegue, avion.vMaxCrucero, ...
        avion.vMaxAproximacion, avion.techoDeVuelo, avion.techoDeVuelo, ...
        min(avion.FWmax, avion.MTOW - avion.OEW - parametrosFijos.PL)];
    
    % Preparación de variables normalizadas (Xref)

    Xref = parametrosFijos.xRef;
    lb_s = lb ./ Xref;
    ub_s = ub ./ Xref;
    
    masterEval = @(X) evaluarVuelo(X, avion, parametrosFijos, fronterasFijas);
    
    % Construcción de matrices de restricciones lineales A*x <= b (Escaladas)
    D  = parametrosFijos.distancia;
    A = zeros(2, control.nvars);
    b = zeros(2, 1);
    A(1, 1:2) = -1; b(1) = -(D - fronterasFijas.x3Max);
    A(2, 1:2) =  1; b(2) =  (D - fronterasFijas.x3Min);
    A_s = A * diag(Xref);
    b_s = b;

    refsEqLin = Xref(2)*ones(2, 1); % Las ecuaciones estas son de orden distancia, luego hay que escalar cada fila tb
    
    A_s = A_s./refsEqLin;
    b_s = b_s./refsEqLin;

    %Escalado de funciones
    % tiempo_min = fronterasFijas.x1Min/(fronterasFijas.vMaxDespegue*cos(fronterasFijas.maxTasaAscenso))...
    %     + (parametrosFijos.distancia-fronterasFijas.x1Min-fronterasFijas.x3Min)/fronterasFijas.vMaxCrucero...
    %     + fronterasFijas.x3Min/(fronterasFijas.vMaxAproximacion*abs(cos(fronterasFijas.maxTasaDescenso)));
    % tiempo_max = parametrosFijos.distancia/2/(fronterasFijas.vMinDespegue + fronterasFijas.vMinAproximacion)*...
    %     cos(fronterasFijas.angulo_min);
    tiempo_min = parametrosFijos.distancia/245;
    tiempo_max = parametrosFijos.distancia/60;
    
    fprintf("Carga de datos completada, comienza la optimización.\n")

    %% 4. OPTIMIZACIÓN HEURÍSTICA (GAMULTIOBJ)
    if control.heuristico
        tic;
        fprintf("Comienza la optimización por algoritmo genético.\n")
        
        %Norma
        objFun_s = @(xs) getOutput(masterEval, xs .* Xref); 
        nonlconFun_s = @(xs) getConstraints(masterEval, xs .* Xref);
        
        optionsGA = optimoptions('gamultiobj', ...
            'UseParallel', true, ...
            'Display', 'iter');
        
        [Xs_ga, F_ga, exitflag_ga, output_ga] = gamultiobj(objFun_s, ...
            control.nvars, A_s, b_s, [], [], lb_s, ub_s, nonlconFun_s, optionsGA);
            
        X_ga = Xs_ga .* Xref; % Des-escalado de resultados
        Resultados.(avionActual).ga.X = X_ga;
        Resultados.(avionActual).ga.F = F_ga;
        Resultados.(avionActual).ga.output = output_ga;
        
        if exist('plotPareto','file'), plotPareto(F_ga, avionActual); end
        
        tiempo = toc;
        fprintf("   Optimización heurística completada. Tiempo de ejecución GA: %.4f s.\n\n", tiempo);
    end
    Xref_nuevo = 1./sqrt(diag(hessiano));
    %% 5. OPTIMIZACIÓN POR GRADIENTE (FMINCON)
    if control.gradiente
        tic;
        fprintf("Comienza la optimización por algoritmo basado en gradiente.\n");
        
        %x0 = [200000, 3600000, 150, 170, 90, 12000, 12000, 5000]; % Punto de partida 
        x0 = X_grad;
        %x0 = [98262.85, 3788456.85, 180.11, 143.31, 121.98, 12746.29, 13155.48, 3529.62]; % Punto de partida después del reescalado (optimo del punto de partida con reescalado intuitivo)
        xs0 = x0 ./ Xref;
        x0 = [1.5e5; 3.94e5; 127.7; 155.4; 130.2; 9.53e3; 1.06e4; 5.43e3];
        %Compruebo que x0 no incumple con las límites
        idx_lb = find(x0 < lb);
        idx_ub = find(x0 > ub);

        funcionCosteEscalar_s = @(xs) sumaPonderada(xs .* Xref, masterEval, control.w1, control.w2, tiempo_min, tiempo_max, ub);
        nonlconFun_s = @(xs) getConstraints(masterEval, xs .* Xref);
       
        optionsGrad = optimoptions('fmincon', ...
            'Display', 'iter', ...
            'UseParallel', true, ...
            'MaxFunctionEvaluations', Inf, ...
            'MaxIterations', Inf, ...
            'OptimalityTolerance', 1e-4, ...
            'StepTolerance', 1e-4, ...
            'ConstraintTolerance', 1e-4, ...
            'FiniteDifferenceType', 'central', ...
            'Algorithm','sqp');
           
        [xs_opt, J_val, exitflag_grad, output_grad, lambda, grad, hessiano] = fmincon(funcionCosteEscalar_s, ...
            xs0, A_s, b_s, [], [], lb_s, ub_s, nonlconFun_s, optionsGrad);
            
     
        X_grad = xs_opt .* Xref;
        F_grad = masterEval(X_grad);
        
        Resultados.(avionActual).grad.X = X_grad;
        Resultados.(avionActual).grad.F = F_grad; 
        Resultados.(avionActual).grad.J = J_val;
        Resultados.(avionActual).grad.output = output_grad;
        F_grad(2)=F_grad(2)*parametrosFijos.PL;
        plotFlight(X_grad, F_grad, avion, parametrosFijos);
        

        %Análisis de sensibilidad de las variables
        % Análisis de sensibilidad de las variables (alrededor del óptimo)
        eps = 1e-3;                          % 0.1% relativo
        dx  = eps * abs(xs_opt);     % evita dx=0
        
        Deltaf_plus  = zeros(control.nvars,2);
        Deltaf_minus = zeros(control.nvars,2);
        dF_dxs = zeros(control.nvars,2);
        
        for i = 1:control.nvars
            xp = xs_opt; xp(i) = xp(i) + dx(i);
            xm = xs_opt; xm(i) = xm(i) - dx(i);
        
            fp = masterEval(xp .* Xref);   % [s; kg/PL]
            fm = masterEval(xm .* Xref);   % [s; kg/PL]
        
            % pasar combustible a kg para comparar con F_grad(2)
            fp(2) = fp(2) * parametrosFijos.PL;
            fm(2) = fm(2) * parametrosFijos.PL;
        
            Deltaf_plus(i,:)  = fp(:).' - F_grad(:).';
            Deltaf_minus(i,:) = fm(:).' - F_grad(:).';

            % Derivada central respecto a xs
            dF_dxs(i,:) = (Deltaf_plus(i,:) - Deltaf_minus(i,:)) / (2*dx(i));
        end
        
        
        % Si quieres derivada respecto a X real:
        dF_dX = dF_dxs ./ Xref(:);

        Xref_nuevo = 1./sqrt(diag(hessiano));
        tiempo = toc;
        fprintf("   Optimización gradiente completada. Tiempo de ejecución Grad: %.4f s.\n\n", tiempo);
    end
end
    
%% 6. GUARDADO DE ARCHIVOS Y SALIDA POR CONSOLA
fprintf(strcat("\nOptimización completada, guardando resultados.\n\n"));
fechaHora = datetime("now", "Format", "yyyyMMdd_HHmm");
nombreArchivo = fullfile("Resultados", "Resultados_" + char(fechaHora) + ".mat");
save(nombreArchivo, 'Resultados');

fprintf('\n======================================================\n');
fprintf(' RESUMEN DE RESULTADOS PARA: %s\n', avionActual);
fprintf('======================================================\n');

if control.gradiente
    fprintf('\n>>> RESULTADOS GRADIENTE (fmincon) <<<\n');
    fprintf('  Exit Flag: %d (1 = Convergencia Exitosa)\n', exitflag_grad);
    fprintf('  Iteraciones: %d\n', output_grad.iterations);
    fprintf('------------------------------------------------------\n');
    fprintf('  COSTO TOTAL (J = w1*f1 + w2*f2): %.6f\n', J_val);
    fprintf('  OBJETIVOS [f1, f2]:              [%.4f,  %.4f]\n', F_grad(1), F_grad(2));
    fprintf('------------------------------------------------------\n');
    fprintf('  VARIABLES DE DISEÑO (X):\n');
    fprintf('    x1-x4:  %.4f  %.4f  %.4f  %.4f\n', X_grad(1:4));
    fprintf('    x5-x8: %.4f  %.4f  %.4f  %.4f\n', X_grad(5:8));
end

if control.heuristico
    fprintf('\n>>> RESULTADOS GENÉTICO (GAMULTIOBJ) <<<\n');
    fprintf('  Puntos encontrados en Pareto: %d\n', size(F_ga, 1));
    fprintf('  Rango f1 (min - max): %.4f - %.4f\n', min(F_ga(:,1)), max(F_ga(:,1)));
    fprintf('  Rango f2 (min - max): %.4f - %.4f\n', min(F_ga(:,2)), max(F_ga(:,2)));
end
fprintf('\n======================================================\n\n');
fprintf('  Numero de condición del hessiano: %.4f\n', cond(hessiano));
fprintf('  Maximo VAP: %.4f\n', max(abs(eig(hessiano))));

%% FUNCIONES AUXILIARES
function f = getOutput(funHandle, x)
    [f, ~, ~] = funHandle(x);
end

function [c, ceq] = getConstraints(funHandle, x)
    [~, c, ceq] = funHandle(x);
end

function J = sumaPonderada(x, funHandle, w1, w2, tiempo_min, tiempo_max, ub)
    [f, ~, ~] = funHandle(x);
   % J = w1 * (f(1)-tiempo_min)/(tiempo_max-tiempo_min) + w2 * f(2)*800/ub(8);
   % J = w1 * (f(1)-tiempo_min)/(tiempo_max-tiempo_min) + w2 * f(2)*800/ub(8);
   J = w1 * f(1) + w2 * f(2);
end
