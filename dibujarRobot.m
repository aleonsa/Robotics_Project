function dibujarRobot(q, parametros, color_options)
% DIBUJARROBOT Dibuja un robot de 3 GDL en la posición especificada
%
%   dibujarRobot(q, parametros, color_options)
%
%   Entradas:
%   - q: Vector con posiciones articulares [q1, q2, q3]
%   - parametros: Estructura con parámetros del robot (l1, l2, l3)
%   - color_options: (Opcional) Estructura con opciones de color

    % Valores por defecto para colores
    if nargin < 3
        color_options = struct('base', [0.5 0.5 0.5], ...
                               'link1', [0.7 0 0], ...
                               'link2', [0.7 0 0], ...
                               'link3', [0 0 1], ...
                               'joint1', [0.2 0.6 1], ...
                               'joint2', [1 1 0], ...
                               'joint3', [1 1 0], ...
                               'ejeX', [1 0 0], ...
                               'ejeY', [0 1 0], ...
                               'ejeZ', [0 0 1]);
    else
        % Asegurarse de que los colores de los ejes están definidos
        if ~isfield(color_options, 'ejeX')
            color_options.ejeX = [1 0 0];
        end
        if ~isfield(color_options, 'ejeY')
            color_options.ejeY = [0 1 0];
        end
        if ~isfield(color_options, 'ejeZ')
            color_options.ejeZ = [0 0 1];
        end
    end
    
    % Extraer parámetros
    l1 = parametros.l1;
    l2 = parametros.l2;
    l3 = parametros.l3;
    
    % Calcular todas las matrices de transformación y posiciones
    % Base a articulación 1
    T01 = [cos(q(1)), 0, sin(q(1)), 0;
           sin(q(1)), 0, -cos(q(1)), 0;
           0, 1, 0, l1;
           0, 0, 0, 1];
    
    % Articulación 1 a 2
    T12 = [cos(q(2)), -sin(q(2)), 0, l2*cos(q(2));
           sin(q(2)), cos(q(2)), 0, l2*sin(q(2));
           0, 0, 1, 0;
           0, 0, 0, 1];
    
    % Articulación 2 a efector final
    T23 = [cos(q(3)), -sin(q(3)), 0, l3*cos(q(3));
           sin(q(3)), cos(q(3)), 0, l3*sin(q(3));
           0, 0, 1, 0;
           0, 0, 0, 1];
    
    % Matrices compuestas
    T02 = T01 * T12;
    T03 = T02 * T23;
    
    % Obtener posiciones
    p0 = [0; 0; 0; 1];  % Base
    p1 = T01 * [0; 0; 0; 1];  % Articulación 1
    p2 = T02 * [0; 0; 0; 1];  % Articulación 2
    p3 = T03 * [0; 0; 0; 1];  % Efector final
    
    % Base (cilindro)
    [X, Y, Z] = cylinder(0.03, 20);
    Z = -Z * 0.175;
    surf(X, Y, Z, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
    
    % Primer eslabón (esfera en la articulación)
    [X, Y, Z] = sphere(20);
    X = X * 0.05;
    Y = Y * 0.05;
    Z = Z * 0.05;
    surf(X, Y, Z, 'FaceColor', [0.2 0.6 1], 'EdgeColor', 'none');
    
    % Dibujar enlaces
    % Enlace 1 (Base a Art1)
    plot3([p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], 'k-', 'LineWidth', 3);
    
    % Enlace 2 (Art1 a Art2)
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], '-', 'Color', color_options.link1, 'LineWidth', 3);
    
    % Enlace 3 (Art2 a Efector)
    plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], '-', 'Color', color_options.link2, 'LineWidth', 3);
    
    % Dibujar articulaciones
    scatter3(p1(1), p1(2), p1(3), 100, color_options.joint1, 'filled');
    scatter3(p2(1), p2(2), p2(3), 100, color_options.joint2, 'filled');
    scatter3(p3(1), p3(2), p3(3), 100, color_options.joint3, 'filled');

     % ----- DIBUJAR SISTEMA DE COORDENADAS DEL EFECTOR FINAL -----
    % Longitud de los ejes del sistema de coordenadas
    longitud_ejes = 0.05;
    
    % Obtener los vectores unitarios de los ejes del efector final desde la matriz T
    eje_x = T03(1:3, 1);  % Primera columna de la matriz de rotación
    eje_y = T03(1:3, 2);  % Segunda columna de la matriz de rotación
    eje_z = T03(1:3, 3);  % Tercera columna de la matriz de rotación
    
    % Posición del efector final
    origen = p3(1:3);
    
    % Dibujar eje X (rojo)
    quiver3(origen(1), origen(2), origen(3), ...
           eje_x(1), eje_x(2), eje_x(3), longitud_ejes, ...
           'Color', color_options.ejeX, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Dibujar eje Y (verde)
    quiver3(origen(1), origen(2), origen(3), ...
           eje_y(1), eje_y(2), eje_y(3), longitud_ejes, ...
           'Color', color_options.ejeY, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Dibujar eje Z (azul)
    quiver3(origen(1), origen(2), origen(3), ...
           eje_z(1), eje_z(2), eje_z(3), longitud_ejes, ...
           'Color', color_options.ejeZ, 'LineWidth', 2, 'MaxHeadSize', 0.5);
end