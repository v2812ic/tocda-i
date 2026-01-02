function [Hdiag_xs, J0] = hessDiagCentralScaled(xs, Xref, Jfun, relStep)
% hessDiagCentralScaled
% Calcula SOLO la diagonal del Hessiano de J respecto a xs (variables escaladas)
%
% xs      : punto en variables escaladas
% Xref    : vector de escalado (x = xs .* Xref)
% Jfun    : handle que acepta x REAL y devuelve escalar J
% relStep : paso relativo (recomendado 1e-3 o 1e-4)
%
% Hdiag_xs : diagonal del Hessiano d^2J/dxs_i^2
% J0       : valor de J en xs

n  = numel(xs);
dx = relStep * max(abs(xs), 1);   % paso relativo robusto en xs

J0 = Jfun(xs .* Xref);

Hdiag_xs = zeros(n,1);

for i = 1:n
    xp = xs; xp(i) = xs(i) + dx(i);
    xm = xs; xm(i) = xs(i) - dx(i);

    Jp = Jfun(xp .* Xref);
    Jm = Jfun(xm .* Xref);

    % Segunda derivada central
    Hdiag_xs(i) = (Jp - 2*J0 + Jm) / (dx(i)^2);
end

end
