% Script de visualización del perfil de vuelo optimizado (VERSIÓN ESBELTA)
clear; clc;

% 1. Configuración de datos de ejemplo
ho = 0;         h1 = 10000;   h2 = 13500;   hd = 5000;
x1 = 250;       x2 = 600;     x3 = 200; 
dist = [0, x1, x1+x2, x1+x2+x3];
alt  = [ho, h1, h2, hd];

% MODIFICACIÓN: Ventana más estrecha y alta (600x800) para efecto esbelto
figure('Color', 'w', 'Position', [200 50 1080 200]);
hold on;

% 2. Dibujar líneas de referencia (Grilla manual sutil)
line([dist(2) dist(2)], [-2000 h1], 'Color', [0.8 0.8 0.8], 'LineStyle', '--'); 
line([dist(3) dist(3)], [-2000 h2], 'Color', [0.8 0.8 0.8], 'LineStyle', '--'); 
line([0 dist(2)], [h1 h1], 'Color', [0.8 0.8 0.8], 'LineStyle', ':');           
line([0 dist(3)], [h2 h2], 'Color', [0.8 0.8 0.8], 'LineStyle', ':');           
% Marca de hd hasta el eje vertical
line([0 dist(4)], [hd hd], 'Color', [0.8 0.8 0.8], 'LineStyle', ':');  

% 3. Dibujar trayectoria principal
plot(dist, alt, 'LineWidth', 2.5, 'Color', [0.1 0.1 0.1]); 
plot(dist, alt, 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k'); 

% 4. Anotaciones de Alturas (Eje Vertical)
% h1 y h2 en AZUL
text(-40, h1, '$h_1$', 'Interpreter', 'latex', 'FontSize', 15, 'Color', 'b', 'HorizontalAlignment', 'right');
text(-40, h2, '$h_2$', 'Interpreter', 'latex', 'FontSize', 15, 'Color', 'b', 'HorizontalAlignment', 'right');
% ho y hd en NEGRO
text(-40, ho, '$h_o$', 'Interpreter', 'latex', 'FontSize', 13, 'Color', 'k', 'HorizontalAlignment', 'right');
text(-40, hd, '$h_d$', 'Interpreter', 'latex', 'FontSize', 13, 'Color', 'k', 'HorizontalAlignment', 'right');

% 5. Velocidades (Variables de decisión -> AZUL)
% V1 separada hacia arriba de la trayectoria
text(dist(1)+x1/2, h1/2 + 4500, '$V_1$', 'Interpreter', 'latex', 'FontSize', 14, 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(dist(2)+x2/2, (h1+h2)/2 + 2400, '$V_2$', 'Interpreter', 'latex', 'FontSize', 14, 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(dist(3)+x3/2, (h2+hd)/2 + 2200, '$V_3$', 'Interpreter', 'latex', 'FontSize', 14, 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 6. Indicadores de Distancia (x1, x2 en AZUL, resto en NEGRO)
y_dim = -2500;
line([0 dist(4)], [y_dim y_dim], 'Color', 'k', 'LineWidth', 1);
line([x1 x1], [y_dim-300 y_dim+300], 'Color', 'k');
line([x1+x2 x1+x2], [y_dim-300 y_dim+300], 'Color', 'k');

text(x1/2, y_dim - 1700, '$x_1$', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', 'b');
text(x1 + x2/2, y_dim - 1700, '$x_2$', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', 'b');
text(x1 + x2 + x3/2, y_dim - 1700, '$d - x_1 - x_2$', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', 'k');

% 7. Títulos de Fases (NEGRO)
y_text = 18000; 
text(x1/2, y_text, '\textbf{ASCENSO}', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', 'k');
text(x1 + x2/2, y_text, '\textbf{CRUCERO}', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', 'k');
text(x1 + x2 + x3/2, y_text, '\textbf{DESCENSO}', 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', 'k');

% 8. Eje Vertical (Grosor fino -> LineWidth 1)
line([0 0], [y_dim y_text], 'Color', 'k', 'LineWidth', 1); 
tick_length = 10;
line([-tick_length 0], [ho ho], 'Color', 'k');
line([-tick_length 0], [h1 h1], 'Color', 'k');
line([-tick_length 0], [h2 h2], 'Color', 'k');
line([-tick_length 0], [hd hd], 'Color', 'k');

% 9. Configuración final
axis off;
% Ajuste de límites para maximizar la verticalidad
xlim([-200 dist(4)+150]); 
ylim([y_dim-3000 y_text+2000]);
hold off;