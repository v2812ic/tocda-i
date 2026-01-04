function plotFlight(x, objetivos, avion, parametrosFijos)
    % x: [d1, d2, v_climb, v_cruc, v_aprox, h1, h2, fuel]

    %% ================= ESCALADO GLOBAL DE FUENTES =================
    fontScale = 1.0;   % <-- AJUSTA AQUÍ (0.8 pequeño | 1.0 normal | 1.2-1.4 grande)
    %% ===============================================================

    %% 1. Preparación de Datos
    d1 = x(1);
    d2 = x(2);
    d3 = parametrosFijos.distancia - (d1 + d2);

    % Coordenadas X (Distancias) en Kilómetros
    dist_acum_m = [0, d1, d1 + d2, parametrosFijos.distancia];
    x_km = dist_acum_m / 1000;

    % Coordenadas Y (Altitudes) en Metros
    h_orig = parametrosFijos.h_origen;
    h1 = x(6);
    h2 = x(7);
    h_dest = parametrosFijos.h_destino;
    y_m = [h_orig, h1, h2, h_dest];

    % Cálculo de ángulos
    gamma1 = atan2d((h1 - h_orig), d1);
    gamma2 = atan2d((h2 - h1), d2);
    gamma3 = atan2d((h_dest - h2), d3);

    %% 2. Configuración de la Figura
    fig = figure('Name', sprintf('Perfil de Vuelo: %s', avion.nombreAvion), ...
                 'Color', 'w', ...
                 'NumberTitle', 'off', ...
                 'Units', 'normalized', 'Position', [0.15 0.15 0.7 0.6]);

    ax = gca;
    hold on;
    grid on; grid minor;

    ax.FontSize = 11 * fontScale;
    ax.FontName = 'Helvetica';
    ax.GridAlpha = 0.3;
    ax.MinorGridAlpha = 0.1;
    ax.LineWidth = 1.2;

    %% 3. Graficado por Segmentos
    col_ascenso  = [0.4660 0.6740 0.1880];
    col_crucero  = [0 0.4470 0.7410];
    col_descenso = [0.8500 0.3250 0.0980];

    lw = 2.5;

    plot(x_km(1:2), y_m(1:2), '-', 'Color', col_ascenso, ...
         'LineWidth', lw, 'DisplayName', 'Fase Ascenso');

    plot(x_km(2:3), y_m(2:3), '-', 'Color', col_crucero, ...
         'LineWidth', lw, 'DisplayName', 'Fase Crucero');

    plot(x_km(3:4), y_m(3:4), '-', 'Color', col_descenso, ...
         'LineWidth', lw, 'DisplayName', 'Fase Descenso');

    plot(x_km, y_m, 'o', ...
         'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', 'w', ...
         'MarkerSize', 7, ...
         'LineWidth', 1.5, ...
         'HandleVisibility', 'off');

    %% 4. Etiquetas y Título
    xlabel('Distancia Recorrida ($km$)', ...
           'Interpreter', 'latex', ...
           'FontSize', 12 * fontScale, ...
           'FontWeight', 'bold');

    ylabel('Altitud ($m$)', ...
           'Interpreter', 'latex', ...
           'FontSize', 12 * fontScale, ...
           'FontWeight', 'bold');

    title_str = sprintf('\\bf Trayectoria: %s \\rm\nTiempo: %.1f min  |  Combustible consumido: %.1f kg', ...
        avion.nombreAvion, objetivos(1)/60, objetivos(2));

    title(title_str, ...
          'FontSize', 14 * fontScale, ...
          'Interpreter', 'tex');

    %% 5. Anotaciones (Text Boxes)
    box_props = {'BackgroundColor', [1 1 1], ...
                 'EdgeColor', [0.8 0.8 0.8], ...
                 'Margin', 5, ...
                 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', ...
                 'Interpreter', 'latex', ...
                 'FontSize', 10 * fontScale};

    mid_d1 = mean(x_km(1:2)); mid_h1 = mean(y_m(1:2));
    mid_d2 = mean(x_km(2:3)); mid_h2 = mean(y_m(2:3));
    mid_d3 = mean(x_km(3:4)); mid_h3 = mean(y_m(3:4));

    y_range = max(y_m) - min(y_m);
    offset = y_range * 0.08;
    offset_x = 0.5;  % en km

    str1 = sprintf('$V_{asc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(3), gamma1);
    text(mid_d1 + offset_x, mid_h1 - offset, str1, box_props{:}, 'Color', col_ascenso*0.8);

    str2 = sprintf('$V_{cruc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(4), gamma2);
    text(mid_d2, mid_h2 - offset*2, str2, box_props{:}, 'Color', col_crucero*0.8);

    str3 = sprintf('$V_{desc}: %.0f$ m/s\n$\\gamma: %.2f^\\circ$', x(5), gamma3);
    text(mid_d3 - offset_x, mid_h3 - offset, str3, box_props{:}, 'Color', col_descenso*0.8);

    %% 6. Ajustes Finales
    axis tight;
    yl = ylim; xl = xlim;
    ylim([yl(1) - y_range*0.15, yl(2) + y_range*0.2]);
    xlim([xl(1), xl(2)]);

    hold off;
end