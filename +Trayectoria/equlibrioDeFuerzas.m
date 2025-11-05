% Inputs:
%   vars        (vector 2x1) Variables a resolver [alpha; T]
%   V           (scalar) Velocidad actual (m/s)
%   h           (scalar) Altitud actual (m)
%   W           (scalar) Peso actual (N)
%   gamma       (scalar) Ángulo de trayectoria actual (rad)
%   avionObj    (objeto) Objeto de la clase 'Avion'
%   rho         (scalar) Densidad del aire actual (kg/m^3)
%
% Outputs:
%   F           (vector 2x1) Residuos de las ecuaciones 
%               - F(1) = L - W*cos(gamma) + T*sin(alpha)
%               - F(2) = Tcos(alpha) - D - W*sin(gamma)