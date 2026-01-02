function Hdiag = hessdiag_fd_X(fun, X, eps_rel)
% Hdiag ≈ diag(∇² f) respecto a X físico
% fun: @(X) escalar
% X  : vector (físico, sin reescalas)
% eps_rel: p.ej. 1e-4 o 1e-3

X = X(:);
n = numel(X);
if nargin < 3, eps_rel = 1e-4; end

% Paso relativo robusto (evita h=0)
h = eps_rel * max(abs(X), 1);

Hdiag = zeros(n,1);
f0 = fun(X);

for i = 1:n
    Xp = X; Xp(i) = Xp(i) + h(i);
    Xm = X; Xm(i) = Xm(i) - h(i);

    fp = fun(Xp);
    fm = fun(Xm);

    Hdiag(i) = (fp - 2*f0 + fm) / (h(i)^2);
end
end
