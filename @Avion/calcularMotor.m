
function [dmdt,Restriccion] = calcularMotor(Avion, estado, T)
%condiciones de vuelo
Alt = estado.h;
z = ISA(Alt); 
Mach = estado.V / (340*sqrt(z));

Restriccion = 0;

% Cambio de los parámetros del motor 
% Fmax=> Maximum sea-level static thrust (N)
% SFC=> Sea-level static TSFC (kg/(N.s))
% tau=> Fastest engine time constant (s) controla lo rápido que se adapta a
% cambios de throttle. PONERLO BAJO PARA QUE CONVERJA
% Nt=> Ratio of installed to unstalled thrust DEJARLO COMO ESTÁ
% ic_source=> initial thrust source ('Internal' o 'External')
% IC=> initial thrust (N) (solo aplica si ic_source = 'Eternal')

% Llamada a la función

TSFC= (0.45+ 0.54*Mach) * sqrt(z);
dmdt = max(0, T*TSFC/3600/9.8);

end


function [z,d] = ISA(h)

        if h <= 11000
     		z = 1-0.0065/288.15*h;
     		d = z^5.255876;
 		else
     		z = 216.65/288.15;
     		d = (22632.1/101325)*exp(-0.0001577*(h-11000));
        end
end
