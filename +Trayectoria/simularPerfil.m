function Resultado = simularPerfil(Dist, vel, altitud, comb, Param, fronteras, Avion)
import Trayectoria.detenerEnDistancia
i = 0;
j = 1;
g = 1;
t0 = 0;
Resultado = struct;
TOW = Avion.OEW+Param.PL+comb;
Resultado.violacionRestricciones = 0;

ptos_ascenso = 10;
ptos_crucero1 = 2;
ptos_ajuste = 2;
ptos_crucero2 = 2;
ptos_descenso = 10;

% Cálculo de distancias
Dist(5) = max(0, Param.distancia - sum(Dist(1:4))); % Protección contra negativos

gamma_f = [atan((altitud(1)-Param.h_origen)/Dist(1)) 0 atan((altitud(2)-altitud(1))/Dist(3)) 0 -atan((altitud(2)-Param.h_destino)/Dist(5))];
% Protección por si Dist es 0 (atan(x/0) da NaN o error)
gamma_f(isnan(gamma_f)) = 0; 

V_asc = vel.*sin(gamma_f);

% Comprobación de restricciones (solo si la fase existe)
% if (Dist(1)>0) && ((gamma_f(1)>fronteras.maxTasaAscenso) || (gamma_f(1) < fronteras.minTasaAscenso))
%     Resultado.violacionRestricciones = 1;
%     g = 8;
% elseif (Dist(3)>0) && ((gamma_f(3)>fronteras.maxTasaAscenso) || (gamma_f(3) < fronteras.minTasaAscenso))
%     Resultado.violacionRestricciones = 1;
%     g = 8;
% elseif (Dist(5)>0) && ((abs(gamma_f(5))>fronteras.maxTasaDescenso) || (abs(gamma_f(5))<fronteras.minTasaDescenso))
%     Resultado.violacionRestricciones = 1;
%     g = 8;
% elseif TOW>Avion.MTOW
%     Resultado.violacionRestricciones = 1;
%     g = 8;
% end

% Tiempo estimado (solo informativo para tspan, protegemos división por cero)
t_fase = Dist./(vel.*cos(gamma_f));
t_fase(isinf(t_fase) | isnan(t_fase)) = 0; 

%Estructura de salida
Ins = struct;

% --- 1 fase ---
i = 1+i;
% Inicialización
Ins.comb_rest(i,j) = comb;
Ins.comb_cons(i,j) = 0;
Ins.dist(i,j) = 0;
Ins.alt(i,j) = Param.h_origen;
Ins.t(i,j) = t0;
Ins.peso(i,j) = TOW;

j = 1+j;

% LOGICA DE SALTO: Si hay distancia, simulamos. Si no, saltamos.
if Dist(i) > 0 
    while g==i
        v = vel(i);
        gamma = gamma_f(i);
        
        [T, alpha, exitflag] = calcularAeroyFuerzas(v, Ins.alt(i,j-1), Ins.peso(i,j-1)*9.81, gamma, Avion, i);

        % Verificamos si la física se rompió
        if exitflag <= 0 || isnan(T) || isnan(alpha)
            % Si fsolve no convergió o devolvió NaNs, abortamos esta simulación
            fprintf('Simulación abortada en fase %d: Error de convergencia.\n', i);
            
            % Marcamos los resultados como fallidos para que el GA lo sepa
            Resultado.violacionRestricciones = 1; 
            g=8;
            break; % Salimos de la función simularPerfil inmediatamente
        end
        
        estado = struct;
        estado.x = Ins.dist(i,j-1); 
        estado.h = Ins.alt(i,j-1);  
        estado.T = T;
        estado.V = v;
        
        [dm,Restriccion_motor] = calcularMotor(Avion, estado, T);

        if Restriccion_motor ==1
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        odefun = dm;
        tspan = [0 t_fase(i)/ptos_ascenso];
        ini = 0;
        
        [t, y] = ode45(@(t,y)odefun, tspan, ini);
        
        Ins.t(i,j) = Ins.t(i,j-1) + t(end);
        Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
        Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
        Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
        Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
        Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
        
        if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        g = detenerEnDistancia(Dist, Ins.dist(i,j));
        j = 1+j;
    end
elseif Dist(i)==0 && g~=8
    % Si la distancia es 0, damos la fase por terminada inmediatamente
    g = i + 1; 
end

Ins.dist_total = Ins.dist;
Ins.t_total = Ins.t;


% --- 2 fase ---
j = 1;
i = 1+i;

idx_ant = find(Ins.peso(i-1,:) > 0, 1, 'last');

Ins.comb_rest(i,j) = Ins.comb_rest(i-1,idx_ant);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,idx_ant);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, idx_ant);
Ins.alt(i,j) = Ins.alt(i-1,idx_ant);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,idx_ant);
Ins.peso(i,j) = Ins.peso(i-1,idx_ant);

j = 1+j;

if Dist(i) > 0
    while g==i
        v = vel(i);
        gamma = gamma_f(i);
        
        [T, alpha, exitflag] = calcularAeroyFuerzas(v, Ins.alt(i,j-1), Ins.peso(i,j-1)*9.81, gamma, Avion, i);

        % Verificamos si la física se rompió
        if exitflag <= 0 || isnan(T) || isnan(alpha)
            % Si fsolve no convergió o devolvió NaNs, abortamos esta simulación
            fprintf('Simulación abortada en fase %d: Error de convergencia.\n', i);
            
            % Marcamos los resultados como fallidos para que el GA lo sepa
            Resultado.violacionRestricciones = 1;  
            g=8;
            break; % Salimos de la función simularPerfil inmediatamente
        end 
        
        estado = struct;
        estado.x = Ins.dist_total(i,j-1); 
        estado.h = Ins.alt(i,j-1);
        estado.T = T;
        estado.V = v; 
        
        [dm,Restriccion_motor] = calcularMotor(Avion, estado, T);

        if Restriccion_motor ==1
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        odefun = dm;
        tspan = [0 t_fase(i)/ptos_crucero1];
        ini = 0;
        
        [t, y] = ode45(@(t,y)odefun, tspan, ini);
        
        Ins.t(i,j) = Ins.t(i,j-1) + t(end);
        Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
        Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
        Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
        Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
        Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
        Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
        Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
        
        if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        g = detenerEnDistancia(Dist, Ins.dist_total(i,j));
        j = 1+j;
    end
elseif Dist(i)==0 && g~=8
    g = i + 1;
end


% --- 3 fase ---
j = 1;
i = 1+i;

idx_ant = find(Ins.peso(i-1,:) > 0, 1, 'last');

Ins.comb_rest(i,j) = Ins.comb_rest(i-1,idx_ant);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,idx_ant);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, idx_ant);
Ins.alt(i,j) = Ins.alt(i-1,idx_ant);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,idx_ant);
Ins.peso(i,j) = Ins.peso(i-1,idx_ant);

j = 1+j;

if Dist(i) > 0
    while g==i
        v = vel(i);
        gamma = gamma_f(i);
        
        [T, alpha, exitflag] = calcularAeroyFuerzas(v, Ins.alt(i,j-1), Ins.peso(i,j-1)*9.81, gamma, Avion, i);

        % Verificamos si la física se rompió
        if exitflag <= 0 || isnan(T) || isnan(alpha)
            % Si fsolve no convergió o devolvió NaNs, abortamos esta simulación
            fprintf('Simulación abortada en fase %d: Error de convergencia.\n', i);
            
            % Marcamos los resultados como fallidos para que el GA lo sepa
            Resultado.violacionRestricciones = 1;  
            g=8;
            break; % Salimos de la función simularPerfil inmediatamente
        end 
        
        estado = struct;
        estado.x = Ins.dist_total(i,j-1);
        estado.h = Ins.alt(i,j-1);
        estado.T = T;
        estado.V = v; 
        
        [dm,Restriccion_motor] = calcularMotor(Avion, estado, T);

        if Restriccion_motor ==1
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        odefun = dm;
        tspan = [0 t_fase(i)/ptos_ajuste];
        ini = 0;
        
        [t, y] = ode45(@(t,y)odefun, tspan, ini);
        
        Ins.t(i,j) = Ins.t(i,j-1) + t(end);
        Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
        Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
        Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
        Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
        Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
        Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
        Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
      
        if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        g = detenerEnDistancia(Dist, Ins.dist_total(i,j));
        j = 1+j;
    end
elseif Dist(i)==0 && g~=8
    g = i + 1;
end


% --- 4 fase ---
j = 1;
i = 1+i;

idx_ant = find(Ins.peso(i-1,:) > 0, 1, 'last');

Ins.comb_rest(i,j) = Ins.comb_rest(i-1,idx_ant);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,idx_ant);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, idx_ant);
Ins.alt(i,j) = Ins.alt(i-1,idx_ant);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,idx_ant);
Ins.peso(i,j) = Ins.peso(i-1,idx_ant);

j = 1+j;

if Dist(i) > 0
    while g==i
        v = vel(i);
        gamma = gamma_f(i);
        
        [T, alpha, exitflag] = calcularAeroyFuerzas(v, Ins.alt(i,j-1), Ins.peso(i,j-1)*9.81, gamma, Avion, i);

        % Verificamos si la física se rompió
        if exitflag <= 0 || isnan(T) || isnan(alpha)
            % Si fsolve no convergió o devolvió NaNs, abortamos esta simulación
            fprintf('Simulación abortada en fase %d: Error de convergencia.\n', i);
            
            % Marcamos los resultados como fallidos para que el GA lo sepa
            Resultado.violacionRestricciones = 1;  
            g=8;
            break; % Salimos de la función simularPerfil inmediatamente
        end 
        
        estado = struct;
        estado.x = Ins.dist_total(i,j-1);
        estado.h = Ins.alt(i,j-1);
        estado.T = T;
        estado.V = v; 
        
        [dm,Restriccion_motor] = calcularMotor(Avion, estado, T);

        if Restriccion_motor ==1
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        odefun = dm;
        tspan = [0 t_fase(i)/ptos_crucero2];
        ini = 0;
        
        [t, y] = ode45(@(t,y)odefun, tspan, ini);
        
        Ins.t(i,j) = Ins.t(i,j-1) + t(end);
        Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
        Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
        Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
        Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
        Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
        Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
        Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
        
        if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        g = detenerEnDistancia(Dist, Ins.dist_total(i,j));
        j = 1+j;
    end
elseif Dist(i)==0 && g~=8
    g = i + 1;
end


% --- 5 fase ---
j = 1;
i = 1+i;

idx_ant = find(Ins.peso(i-1,:) > 0, 1, 'last');

Ins.comb_rest(i,j) = Ins.comb_rest(i-1,idx_ant);
Ins.comb_cons(i,j) = Ins.comb_cons(i-1,idx_ant);
Ins.dist(i,j) = 0;
Ins.dist_total(i,j) = Ins.dist_total(i-1, idx_ant);
Ins.alt(i,j) = Ins.alt(i-1,idx_ant);
Ins.t(i,j) = 0;
Ins.t_total(i,j) = Ins.t_total(i-1,idx_ant);
Ins.peso(i,j) = Ins.peso(i-1,idx_ant);

j = 1+j;

if Dist(i) > 0
    while g==i
        v = vel(i);
        gamma = gamma_f(i);
        
        [T, alpha, exitflag] = calcularAeroyFuerzas(v, Ins.alt(i,j-1), Ins.peso(i,j-1)*9.81, gamma, Avion, i);

        % Verificamos si la física se rompió
        if exitflag <= 0 || isnan(T) || isnan(alpha)
            % Si fsolve no convergió o devolvió NaNs, abortamos esta simulación
            fprintf('Simulación abortada en fase %d: Error de convergencia.\n', i);
            
            % Marcamos los resultados como fallidos para que el GA lo sepa
            Resultado.violacionRestricciones = 1; 
            g=8;
            break % Salimos de la función simularPerfil inmediatamente
        end
        
        estado = struct;
        estado.x = Ins.dist_total(i,j-1);
        estado.h = Ins.alt(i,j-1);
        estado.T = T;
        estado.V = v; 
        
        [dm,Restriccion_motor] = calcularMotor(Avion, estado, T);

        if Restriccion_motor ==1
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        odefun = dm;
        tspan = [0 t_fase(i)/ptos_descenso];
        ini = 0;
        
        [t, y] = ode45(@(t,y)odefun, tspan, ini);
        
        Ins.t(i,j) = Ins.t(i,j-1) + t(end);
        Ins.t_total(i,j) = Ins.t_total(i,1)+Ins.t(i,j);
        Ins.comb_cons(i,j) = Ins.comb_cons(i,j-1) + y(end);
        Ins.comb_rest(i,j) = Ins.comb_rest(i,j-1) - y(end);
        Ins.dist(i,j) = v*Ins.t(i,j)*cos(gamma);
        Ins.dist_total(i,j) = Ins.dist_total(i,1) + Ins.dist(i,j);
        Ins.alt(i,j) = Ins.alt(i,1) + v*Ins.t(i,j)*sin(gamma);
        Ins.peso(i,j) = Ins.peso(i,j-1) - y(end);
        
        if Ins.comb_cons(i,j)>(comb/Param.seguridadFuel)
            Resultado.violacionRestricciones = 1;
            g=8;
            break
        end
        
        g = detenerEnDistancia(Dist, Ins.dist_total(i,j));
        j = 1+j;
    end
elseif Dist(i)==0 && g~=8
    g = i + 1;
end

% Búsqueda segura del final
idx_final = find(Ins.t_total(i,:) > 0, 1, 'last');
if isempty(idx_final)
    Resultado.tiempoTotal = 0;
    Resultado.combustibleConsumido = 0;
else
    Resultado.tiempoTotal = Ins.t_total(i, idx_final);
    Resultado.combustibleConsumido = Ins.comb_cons(i,idx_final);
end
%Resultado.violacionRestricciones
end

