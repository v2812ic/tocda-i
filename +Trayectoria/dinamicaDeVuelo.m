% Inputs:
%   t           (scalar) Tiempo actual (inyectado por ode45)
%   S           (vector 3x1) Vector de estado actual [x; h; m] (inyectado por ode45)
%   V_fase      (scalar) Velocidad objetivo para esta fase (m/s)
%   gamma_fase  (scalar) Ángulo de trayectoria para esta fase (rad)
%   avionObj    (objeto) Objeto de la clase 'Avion'
%
% Outputs:
%   dS_dt       (vector 3x1) Derivadas del estado [dx/dt; dh/dt; dm/dt]
%               - dx/dt = V*cos(gamma)
%               - dh/dt = V*sin(gamma)
%               - dm/dt = -m_dot (consumo de combustible)