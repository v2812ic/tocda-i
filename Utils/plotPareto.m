function [] = plotPareto(F, avionActual)
% Grafica el frente de pareto para los resultados F_ga de la función de
% optimización, y el nombre del avion avionActual
    
    titulo = strcat("Frente de Pareto para ", avionActual);

    figure; 
    scatter(F(:, 1), F(:, 2), 'filled');
    
    xlabel('Objetivo 1: Tiempo Total [s]');
    ylabel('Objetivo 2: Consumo de Combustible [kg]');
    
    title(titulo);
    grid on;
    
end