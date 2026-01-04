function x0 = DoE(lb, ub, avion, parametrosFijos, fronterasFijas)

    import Core.evaluarVuelo

    % lb = [min(fronterasFijas.x1Min, parametrosFijos.distancia), min(fronterasFijas.x2Min, parametrosFijos.distancia), avion.vMinDespegue, avion.vMinCrucero, ...
    %     avion.vMinAproximacion, fronterasFijas.hCruceroMin, fronterasFijas.hCruceroMin,...
    %     0];
    % ub = [min(fronterasFijas.x1Max, parametrosFijos.distancia), min(fronterasFijas.x2Max, parametrosFijos.distancia), avion.vMaxDespegue, avion.vMaxCrucero, ...
    %     avion.vMaxAproximacion, avion.techoDeVuelo, avion.techoDeVuelo, ...
    %     min(avion.FWmax, avion.MTOW - avion.OEW - parametrosFijos.PL)];

    factores = 8;
    experimentos = 100;
    rng(1);
    X_lhs = lhsdesign(experimentos, factores);

    X = lb + X_lhs .* (ub - lb);

    masterEval = @(X) evaluarVuelo(X, avion, parametrosFijos, fronterasFijas);
    
    parfor i = 1:100

    Y(:,i) = masterEval(X(i,:));
    
    end
    R = corr(X, Y');

    p = 0.1; % 10% mejores puntos
    [~,idx] = sort(Y);
    best = X(idx(1:round(p*length(Y))), :);
    x0 = mean(best,1);


end














