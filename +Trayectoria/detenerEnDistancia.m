% Evento para parar la ode45 al llegar a una distancia objetivo
% Input: 
%   - t
%   - estado (h, v, w, x)
%   - distanciaObj
% Output:
%   - value x - distanciaObj
%   
function [g] = detenerEnDistancia(DistFase, DistRec)

    if DistRec < DistFase(1)
        g = 1;
    elseif DistRec < (DistFase (1) + DistFase(2))
        g = 2;
    elseif DistRec < (DistFase (1) + DistFase(2)+DistFase(3))
        g=3;
    elseif DistRec < (DistFase (1) + DistFase(2)+DistFase(3)+DistFase(4))
        g=4;
    elseif DistRec < (DistFase (1) + DistFase(2)+DistFase(3)+DistFase(4)+DistFase(5))
        g=5;
    else
        g=6;
    end
end
%  
