function plotFlight(x, objetivos, avion, parametrosFijos)
    % x: [d1, d2, v_climb, v_cruc, v_aprox, h1, h2, fuel]
    
    % 1. Reconstrucción de distancias
    d1 = x(1);
    d2 = x(2);
    d3 = parametrosFijos.distancia - (d1 + d2);
    dist_acum = [0, d1, d1 + d2, parametrosFijos.distancia];
    
    % 2. Reconstrucción de altitudes
    h_orig = parametrosFijos.h_origen;
    h1 = x(6);
    h2 = x(7);
    h_dest = parametrosFijos.h_destino;
    y_coords = [h_orig, h1, h2, h_dest];
    x_coords = dist_acum; 
    
    % --- 3. Cálculo de Ángulos de Pendiente (Gamma) ---
    % Usamos atan2d(delta_h, delta_d) para obtener grados
    gamma1 = atan2d((h1 - h_orig), d1);
    gamma2 = atan2d((h2 - h1), d2);
    gamma3 = atan2d((h_dest - h2), d3);
    
    % 4. Gráfico
    figure('Color', 'w', 'Name', sprintf('Perfil de Vuelo - ', avion.nombreAvion), 'Units', 'normalized', 'Position', [0.2 0.2 0.6 0.5]);
    hold on;
    
    % Dibujar el perfil con un estilo más limpio
    plot(x_coords/1000, y_coords, 'Color', [0 0.447 0.741], 'LineWidth', 2.5);
    plot(x_coords/1000, y_coords, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    
    % Estética
    grid on; ax = gca; ax.GridAlpha = 0.2;
    xlabel('Distancia Recorrida (km)', 'FontSize', 11);
    ylabel('Altitud (m)', 'FontSize', 11);
    title(sprintf('Trayectoria Optimizada: %s\nTiempo: %.2f min | Consumo: %.2f kg', ...
        avion.nombreAvion, objetivos(1)/60, objetivos(2)), 'FontSize', 13);
    
    % 5. Anotaciones de Velocidad y ÁNGULOS
    % Usamos un offset para que no se solapen los textos
    offset_v = 400;
    offset_angle = -500; % Por debajo de la línea
    
    % Fase 1: Ascenso
    text(d1/2000, mean([h_orig, h1]) + offset_v, ...
        sprintf('V_{climb}: %.0f m/s', x(3)), 'Color', [0.5 0 0], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(d1/2000, mean([h_orig, h1]) + offset_angle, ...
        sprintf('\\gamma: %.2f°', gamma1), 'Color', [0.2 0.2 0.2], 'HorizontalAlignment', 'center');
    
    % Fase 2: Crucero
    text((d1 + d2/2)/1000, h1 + offset_v, ...
        sprintf('V_{cruc}: %.0f m/s', x(4)), 'Color', [0.5 0 0], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text((d1 + d2/2)/1000, h1 + offset_angle, ...
        sprintf('\\gamma: %.2f°', gamma2), 'Color', [0.2 0.2 0.2], 'HorizontalAlignment', 'center');
    
    % Fase 3: Descenso
    text((d1 + d2 + d3/2)/1000, mean([h2, h_dest]) + offset_v, ...
        sprintf('V_{aprox}: %.0f m/s', x(5)), 'Color', [0.5 0 0], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text((d1 + d2 + d3/2)/1000, mean([h2, h_dest]) + offset_angle, ...
        sprintf('\\gamma: %.2f°', gamma3), 'Color', [0.2 0.2 0.2], 'HorizontalAlignment', 'center');
    
    % Ajustar límites para que el texto no se corte
    ylim([min(y_coords)-1000, max(y_coords)+1500]);
    
    hold off;
end