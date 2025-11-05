% Inputs:
%   V           (scalar) Velocidad actual (m/s)
%   h           (scalar) Altitud actual (m)
%   W           (scalar) Peso actual (N)
%   gamma       (scalar) Ángulo de trayectoria actual (rad)
%   avionObj    (objeto) Objeto de la clase 'Avion'
%   rho         (scalar) Densidad del aire actual (kg/m^3)
%
% Outputs:
%  -  angulo de ataque correspondiente alpha
%  -  empuje del motor correspondiente.


%        Ecuaciones:
%                0 = L - W*cos(gamma) + T*sin(alpha)
%                0 = Tcos(alpha) - D - W*sin(gamma)

% Estas ecuaciones hay que resolverlas numéricamente ya que L y D son
% funciones de alpha. Estos L y D en función de alpha se sacan de
% Avion.calcularAero