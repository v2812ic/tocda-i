% Inputs:
%   obj         (objeto) La instancia de la clase 'Avion' (inyectado)
%   V           (scalar) Velocidad actual (m/s)
%   h           (scalar) Altitud actual (m)
%   T           (scalar) empuje deseado
%
% Outputs:
%   m_dot       (scalar) Flujo de combustible (kg/s)

function dmdt = calcularMotor(avion)

% a partir del estado del avion calcular el consumo instantáneo de
% combustible

dmdt = FuelFlow_Turbofan(0.8, 11000, 100, opts)

end

