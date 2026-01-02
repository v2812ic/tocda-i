function plotFlight(x, objetivos, avion, parametrosFijos)
    % x: [d1, d2, v_climb, v_cruc, v_aprox, h1, h2, fuel]
    
    %% 1. Preparación de Datos
    d1 = x(1);
    d2 = x(2);
    d3 = parametrosFijos.distancia - (d1 + d2);
    
    % Coordenadas X (Distancias) en Kilómetros para graficar
    dist_acum_m = [0, d1, d1 + d2, parametrosFijos.distancia];
    x_km = dist_acum_m / 1000; 
    
    % Coordenadas Y (Altitudes) en Metros
    h_orig = parametrosFijos.h_origen;
    h1 = x(6);
    h2 = x(7);
    h_dest = parametrosFijos.h_destino;
    y_m = [h_orig, h1, h2, h_dest];
    
    % Cálculos de Ángulos
    gamma1 = atan2d((h1 - h_orig), d1);
    gamma2 = atan2d((h2 - h1), d2);
    gamma3 = atan2d((h_dest - h2), d3);
    
    %% 2. Configuración de la Figura
    % Usamos un tema oscuro suave o blanco limpio. Aquí blanco profesional.
    fig = figure('Name', sprintf('Perfil de Vuelo: ', avion.nombreAvion), ...
                 'Color', 'w', ...
                 'NumberTitle', 'off', ...
                 'Units', 'normalized', 'Position', [0.15 0.15 0.7 0.6]);
    
    ax = gca;
    hold on;
    grid on; grid minor;
    ax.FontSize = 11;
    ax.FontName = 'Helvetica'; % Fuente más limpia
    ax.GridAlpha = 0.3;
    ax.MinorGridAlpha = 0.1;
    ax.LineWidth = 1.2;
    
    %% 3. Graficado por Segmentos (Colores Distintos)
    % Definir paleta de colores (RGB)
    col_ascenso  = [0.4660 0.6740 0.1880]; % Verde
    col_crucero  = [0 0.4470 0.7410];      % Azul
    col_descenso = [0.8500 0.3250 0.0980]; % Naranja/Rojo
    
    lw = 2.5; % Grosor de línea
    
    % Segmento 1: Ascenso
    plot(x_km(1:2), y_m(1:2), '-', 'Color', col_ascenso, 'LineWidth', lw, 'DisplayName', 'Fase Ascenso');
    % Segmento 2: Crucero
    plot(x_km(2:3), y_m(2:3), '-', 'Color', col_crucero, 'LineWidth', lw, 'DisplayName', 'Fase Crucero');
    % Segmento 3: Descenso
    plot(x_km(3:4), y_m(3:4), '-', 'Color', col_descenso, 'LineWidth', lw, 'DisplayName', 'Fase Descenso');
    
    % Marcadores en los waypoints (Círculos rellenos)
    p_markers = plot(x_km, y_m, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineWidth', 1.5, 'HandleVisibility', 'off');

    %% 4. Decoración y Etiquetas
    xlabel('Distancia Recorrida ($km$)', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Altitud ($m$)', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Título con información clave
    title_str = sprintf('\\bf Trayectoria: %s \\rm\nTiempo: %.1f min  |  Fuel: %.1f kg', ...
        avion.nombreAvion, objetivos(1)/60, objetivos(2));
    title(title_str, 'FontSize', 14, 'Interpreter', 'tex');
    
    
    %% 5. Anotaciones Inteligentes (Text Boxes)
    % Función auxiliar para formatear las cajas de texto
    box_props = {'BackgroundColor', [1 1 1], 'EdgeColor', [0.8 0.8 0.8], 'Margin', 5, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Interpreter', 'latex'};
    
    % Punto medio de cada segmento para poner el texto
    mid_d1 = (x_km(1) + x_km(2))/2;
    mid_h1 = (y_m(1) + y_m(2))/2;
    
    mid_d2 = (x_km(2) + x_km(3))/2;
    mid_h2 = (y_m(2) + y_m(3))/2;
    
    mid_d3 = (x_km(3) + x_km(4))/2;
    mid_h3 = (y_m(3) + y_m(4))/2;
    
    % Calculamos un offset dinámico basado en la escala actual
    y_range = max(y_m) - min(y_m);
    offset = y_range * 0.08; % 8% del rango vertical
    offset_x = 500;
    
    % -- Fase 1 --
    str1 = sprintf('$V_{asc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(3), gamma1);
    text(mid_d1 + offset_x, mid_h1 - offset, str1, box_props{:}, 'Color', col_ascenso*0.8);
    
    % -- Fase 2 --
    str2 = sprintf('$V_{cruc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(4), gamma2);
    text(mid_d2, mid_h2 - offset * 2, str2, box_props{:}, 'Color', col_crucero*0.8);
    
    % -- Fase 3 --
    str3 = sprintf('$V_{desc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(5), gamma3);
    text(mid_d3 - offset_x, mid_h3 - offset, str3, box_props{:}, 'Color', col_descenso*0.8);
    
    %% 6. Ajustes Finales
    % Añadir un poco de margen a los ejes para que no se corte nada
    axis tight;
    yl = ylim; xl = xlim;
    ylim([yl(1) - y_range*0.15, yl(2) + y_range*0.2]); % Más margen arriba y abajo
    xlim([xl(1), xl(2)]);
    
    hold off;
end